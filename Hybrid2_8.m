%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.8
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

%% Loading previously defined data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;

%% Pload
for k = 1:100
    if k <= 20
        Pload(k) = 0;
    elseif k >= 21 && k <= 50
        Pload(k) = 30+2*k;
    elseif k >= 51
        Pload(k) = 45;
    end
end

%% Constructing M matrices for update equation MILP
% M1.M1 matrix for the diesel generator
M1.M1_d_delta = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B2,2));
M1.M1_d_u = zeros(dim.Np,dim.Np);
% TO DO: NOW M1.M1_d_u IS A ZEROS MATRIX BUT MAKE IT GENERAL!
M1.M1_d_zd = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B3,2));

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1.M1_d_delta(1+(np2-1)*size(MLDD.A,1),1+(np1-1)*size(MLDD.B2,2):np1*size(MLDD.B2,2))  = MLDD.A^(np2-1)*MLDD.B2;
    M1.M1_d_u(1+(np2-1)*size(MLDD.A,1),1+(np1-1):np1*size(MLDD.B1,2))                      = MLDD.A^(np2-1)*MLDD.B1;
    M1.M1_d_zd (1+(np2-1)*size(MLDD.A,1),1+(np1-1)*size(MLDD.B3,2):np1*size(MLDD.B3,2))    = MLDD.A^(np2-1)*MLDD.B3;
    end
end
clear np1

for i = 1:dim.Np
    for j = 1:4*dim.Np
        if j > 4*i
        M1.M1_d_delta(i,j) = 0;
        M1.M1_d_zd(i,j) = 0;
        end
    end
end
clear i j

M1.M1_d = [M1.M1_d_delta M1.M1_d_u M1.M1_d_zd];

% M1.M1 matrix for the batteries
M1.M1_b1_delta = zeros(dim.Np,dim.Np);
M1.M1_b1_u     = zeros(dim.Np,dim.Np);
M1.M1_b1_zd    = zeros(dim.Np,dim.Np);
M1.M1_b1       = zeros(dim.Np,3*dim.Np);

M1.M1_b2_delta = zeros(dim.Np,dim.Np);
M1.M1_b2_u     = zeros(dim.Np,dim.Np);
M1.M1_b2_zd    = zeros(dim.Np,dim.Np);
M1.M1_b2       = zeros(dim.Np,3*dim.Np);

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1.M1_b1_delta(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B2,2):np1*size(MLDB1.B2,2))  = MLDB1.A^(np2-1)*MLDB1.B2;
    M1.M1_b1_u(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B3,2):np1*size(MLDB1.B3,2))      = MLDB1.A^(np2-1)*MLDB1.B1;
    M1.M1_b1_zd(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B1,2):np1*size(MLDB1.B1,2))     = MLDB1.A^(np2-1)*MLDB1.B3;
    
    M1.M1_b2_delta(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B2,2):np1*size(MLDB2.B2,2))  = MLDB2.A^(np2-1)*MLDB2.B2;
    M1.M1_b2_u(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B3,2):np1*size(MLDB2.B3,2))      = MLDB2.A^(np2-1)*MLDB2.B1;
    M1.M1_b2_zd(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B1,2):np1*size(MLDB2.B1,2))     = MLDB2.A^(np2-1)*MLDB2.B3;
    end
end
clear np1 np2

% NOTE: MAYBE THIS CAN BE MADE MORE GENERAL (BUT IT'S A LOT OF WORK AND NOT
% THAT NECESSARY)
for i = 1:dim.Np
    for j = 1:dim.Np
        if j > i
        M1.M1_b1_delta(i,j) = 0;
        M1.M1_b1_u(i,j)     = 0;
        M1.M1_b1_zd(i,j)    = 0;
        
        M1.M1_b2_delta(i,j) = 0;
        M1.M1_b2_u(i,j)     = 0;
        M1.M1_b2_zd(i,j)    = 0;
        end
    end
end
clear i j

M1.M1_b1 = [M1.M1_b1_delta M1.M1_b1_u M1.M1_b1_zd];
M1.M1_b2 = [M1.M1_b2_delta M1.M1_b2_u M1.M1_b2_zd];

% Total M1.M1 matrix
M1.M1 = [M1.M1_b1 zeros(dim.Np,size(M1.M1_b2,2)) zeros(dim.Np,size(M1.M1_d,2)); zeros(dim.Np,size(M1.M1_b1,2)) M1.M1_b2 zeros(dim.Np,size(M1.M1_d,2)); zeros(dim.Np,size(M1.M1_b1,2)) zeros(dim.Np,size(M1.M1_b2,2)) M1.M1_d];

% M2.M2 matrix
M2.M2_b1 = zeros(dim.Np,size(MLDB1.A,2));
M2.M2_b2 = zeros(dim.Np,size(MLDB1.A,2));
M2.M2_d  = zeros(dim.Np,size(MLDD.A,2));
%TO DO: FIX THAT FOR A DIFFERENT LENGTH OF A, THE M2.M2 MATRIX IS FILLED
%CORRECTLY

for np = 1:dim.Np
    M2.M2_b1(np,:) = (MLDB1.A)^np; 
    M2.M2_b2(np,:) = (MLDB2.A)^np;
    M2.M2_d(np,:)  = (MLDD.A)^np;
end
clear np

M2.M2 = [M2.M2_b1 zeros(size(M2.M2_b1,1),size(M2.M2_b2,2)) zeros(size(M2.M2_b1,1),size(M2.M2_d,2)); ...
         zeros(size(M2.M2_b2,1),size(M2.M2_b1,2)) M2.M2_b2 zeros(size(M2.M2_b2,1),size(M2.M2_d,2)); ...
         zeros(size(M2.M2_d,1),size(M2.M2_b1,2)) zeros(size(M2.M2_d,1),size(M2.M2_b2,2)) M2.M2_d];

% M3.M3 matrix
M3.M3_b1 = zeros(dim.Np*size(MLDD.B4,1),size(MLDD.B4,2));
M3.M3_b2 = zeros(dim.Np*size(MLDD.B4,1),size(MLDD.B4,2));
M3.M3_d  = zeros(dim.Np*size(MLDD.B4,1),size(MLDD.B4,2));

for nd = 1:dim.Np
    if nd == 1
        M3.M3_d(1,1) = MLDD.B4;
    end
   
    if nd > 1
        M3.M3_d(1+(nd-1)*size(MLDD.B4,1):nd*size(MLDD.B4,1),1:size(MLDD.B4,2)) = M3.M3_d(nd-1,:) + MLDD.B4*(MLDD.A)^(nd-1);
    end    
end
clear nd

% Concatenating submatrices into complete M3.M3 matrix
M3.M3 = [M3.M3_b1; M3.M3_b2; M3.M3_d];
      
%% Constructing F1.F1 matrix for MILP constraint equation
F1.F1b1_delta = zeros(dim.Np*size(MLDB1.E3,1),dim.Np*size(MLDB1.E3,2));
F1.F1b1_u = zeros(dim.Np*size(MLDB1.E2,1),dim.Np*size(MLDB1.E2,2));
F1.F1b1_zd = zeros(dim.Np*size(MLDB1.E4,1),dim.Np*size(MLDB1.E4,2));

F1.F1b2_delta = zeros(dim.Np*size(MLDB2.E3,1),dim.Np*size(MLDB2.E3,2));
F1.F1b2_u = zeros(dim.Np*size(MLDB2.E2,1),dim.Np*size(MLDB2.E2,2));
F1.F1b2_zd = zeros(dim.Np*size(MLDB2.E4,1),dim.Np*size(MLDB2.E4,2));

F1.F1d_delta = zeros(dim.Np*size(MLDD.E3,1),dim.Np*size(MLDD.E3,2));
F1.F1d_u = zeros(dim.Np*size(MLDD.E2,1),dim.Np*size(MLDD.E2,2));
F1.F1d_zd = zeros(dim.Np*size(MLDD.E3,1),dim.Np*size(MLDD.E4,2));

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
        if np1 == np2
        % For battery 1
        F1.F1b1_delta(1+(np2-1)*size(MLDB1.E3,1):np2*size(MLDB1.E3,1),1+(np1-1)*size(MLDB1.E3,2):np1*size(MLDB1.E3,2))    = MLDB1.E3;
        F1.F1b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2))        = MLDB1.E2;
        F1.F1b1_zd(1+(np2-1)*size(MLDB1.E4,1):np2*size(MLDB1.E4,1),1+(np1-1)*size(MLDB1.E4,2):np1*size(MLDB1.E4,2))       = MLDB1.E4;
        
        % For battery 2
        F1.F1b2_delta(1+(np2-1)*size(MLDB2.E3,1):np2*size(MLDB2.E3,1),1+(np1-1)*size(MLDB2.E3,2):np1*size(MLDB2.E3,2))    = MLDB2.E3;
        F1.F1b2_u(1+(np2-1)*size(MLDB2.E2,1):np2*size(MLDB2.E2,1),1+(np1-1)*size(MLDB2.E2,2):np1*size(MLDB2.E2,2))        = MLDB2.E2;
        F1.F1b2_zd(1+(np2-1)*size(MLDB2.E4,1):np2*size(MLDB2.E4,1),1+(np1-1)*size(MLDB2.E4,2):np1*size(MLDB2.E4,2))       = MLDB2.E4;
        
        % For the diesel generator
        F1.F1d_delta(1+(np2-1)*size(MLDD.E3,1):np2*size(MLDD.E3,1),1+(np1-1)*size(MLDD.E3,2):np1*size(MLDD.E3,2)) = MLDD.E3;
        F1.F1d_u(1+(np2-1)*size(MLDD.E2,1):np2*size(MLDD.E2,1),1+(np1-1)*size(MLDD.E2,2):np1*size(MLDD.E2,2))     = MLDD.E2;
        F1.F1d_zd(1+(np2-1)*size(MLDD.E4,1):np2*size(MLDD.E4,1),1+(np1-1)*size(MLDD.E4,2):np1*size(MLDD.E4,2))    = MLDD.E4;
        end
        
        if np2 > np1 % to fill only below the block diagonal
        % NOTE: algemener maken voor F1.F1_bi_delta?
        % For battery 1
        F1.F1b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2))  = MLDB1.E1*MLDB1.A^(np2-2)*MLDB1.B1;
        F1.F1b1_zd(1+(np2-1)*size(MLDB1.E4,1):np2*size(MLDB1.E4,1),1+(np1-1)*size(MLDB1.E4,2):np1*size(MLDB1.E4,2)) = MLDB1.E1*MLDB1.A^(np2-2)*MLDB1.B3;
        
        % For battery 2
        F1.F1b2_u(1+(np2-1)*size(MLDB2.E2,1):np2*size(MLDB2.E2,1),1+(np1-1)*size(MLDB2.E2,2):np1*size(MLDB2.E2,2))  = MLDB2.E1*MLDB2.A^(np2-2)*MLDB2.B1;
        F1.F1b2_zd(1+(np2-1)*size(MLDB2.E4,1):np2*size(MLDB2.E4,1),1+(np1-1)*size(MLDB2.E4,2):np1*size(MLDB2.E4,2)) = MLDB2.E1*MLDB2.A^(np2-2)*MLDB2.B3;
        
        % For the diesel generator
        F1.F1d_delta(1+(np2-1)*size(MLDD.E3,1):np2*size(MLDD.E3,1),1+(np1-1)*size(MLDD.E3,2):np1*size(MLDD.E3,2)) = MLDD.E1*MLDD.A^(np2-2)*MLDD.B2;
        F1.F1d_zd(1+(np2-1)*size(MLDD.E4,1):np2*size(MLDD.E4,1),1+(np1-1)*size(MLDD.E4,2):np1*size(MLDD.E4,2)) = MLDD.E1*MLDD.A^(np2-2)*MLDD.B3;
        end
    end
end
clear np1 np2

%% Concatenating all submatrices
F1.F1b1 = [F1.F1b1_delta F1.F1b1_u F1.F1b1_zd];
F1.F1b2 = [F1.F1b2_delta F1.F1b2_u F1.F1b2_zd];
F1.F1d = [F1.F1d_delta F1.F1d_u F1.F1d_zd];
F1.F1 = [F1.F1b1 zeros(size(F1.F1b1,1),size(F1.F1b2,2)) zeros(size(F1.F1b1,1),size(F1.F1d,2));
         zeros(size(F1.F1b2,1),size(F1.F1b1,2)) F1.F1b2 zeros(size(F1.F1b2,1),size(F1.F1d,2))
         zeros(size(F1.F1d,1),size(F1.F1b1,2)) zeros(size(F1.F1d,1),size(F1.F1b2,2)) F1.F1d];
     
%% Constructing F1 matrices for Pimp
F1.F11b1 = zeros(dim.Np,3*dim.Np);
F1.F11d  = zeros(dim.Np,9*dim.Np);

% F1 for the batteries
diag = [0 1 0];
F1.F11b1 = kron(eye(dim.Np),diag);
F1.F11b2 = F1.F11b1;

% F1 for the diesel generator
diagd = [0 0 0 0 1 0 0 0 0];
F1.F11d = kron(eye(dim.Np),diagd);

F1.F1new = [F1.F1 zeros(size(F1.F1,1),dim.Np);
            F1.F11b1 F1.F11b2 F1.F11d eye(dim.Np);
            -F1.F11b1 -F1.F11b2 -F1.F11d -eye(dim.Np)];

%% Constructing F2.F2 matrix for MILP constraint equation
F2.F2b1   = zeros(dim.Np*size(MLDB1.g5,1),size(MLDB1.g5,2));
F2.F2b2   = zeros(dim.Np*size(MLDB2.g5,1),size(MLDB2.g5,2));
F2.F2d_1  = zeros(dim.Np*size(MLDD.g5,1),size(MLDD.g5,2));
F2.F2d_2  = zeros(dim.Np*size(MLDD.g5,1),size(MLDD.g5,2));

for n = 1:dim.Np
    F2.F2b1(1+(n-1)*size(MLDB1.g5,1):n*size(MLDB1.g5,1),1:size(MLDB1.g5,2)) = MLDB1.g5;
    F2.F2b2(1+(n-1)*size(MLDB2.g5,1):n*size(MLDB2.g5,1),1:size(MLDB2.g5,2)) = MLDB2.g5;
    F2.F2d_1(1+(n-1)*size(MLDD.g5,1):n*size(MLDD.g5,1),1:size(MLDD.g5,2)) = MLDD.g5;
    
    if n == 2
        F2.F2d_2(1+(n-1)*size(MLDD.E1):n*size(MLDD.E1,1),1:size(MLDD.g5,2)) = MLDD.E1*MLDD.B4;
    end
   
    if n > 2
        F2.F2d_2(1+(n-1)*size(MLDD.E1):n*size(MLDD.E1,1),1:size(MLDD.g5,2)) = F2.F2d_2(1+(n-2)*size(MLDD.E1,1):(n-1)*size(MLDD.E1,1)) + MLDD.E1*MLDD.B4*(MLDD.A)^(n-2);
    end
    
end
clear n

% Concatenating submatrices into complete F2.F2 matrix
F2.F2d = F2.F2d_1 - F2.F2d_2;
F2.F2 = [F2.F2b1; F2.F2b2; F2.F2d];

%% Constructing F2 for Pimp
F2.F2new = [F2.F2;
            Pload(1:dim.Np)';
            -Pload(1:dim.Np)'];

%% Constructing F3.F3 matrix for MILP constraint equation
F3.F3b1 = zeros(dim.Np*size(MLDB1.E1,1),size(MLDB1.E1,2));
F3.F3b2 = zeros(dim.Np*size(MLDB2.E1,1),size(MLDB2.E1,2));
F3.F3d  = zeros(dim.Np*size(MLDD.E1,1),size(MLDD.E1,2));

for n = 1:dim.Np
    F3.F3b1(1+(n-1)*size(MLDB1.E1,1):n*size(MLDB1.E1,1),1:size(MLDB1.E1,2)) = -MLDB1.E1*MLDB1.A^(n-1);
    F3.F3b2(1+(n-1)*size(MLDB2.E1,1):n*size(MLDB2.E1,1),1:size(MLDB2.E1,2)) = -MLDB2.E1*MLDB2.A^(n-1);
    F3.F3d(1+(n-1)*size(MLDD.E1,1):n*size(MLDD.E1,1),1:size(MLDD.E1,2))     = -MLDD.E1*MLDD.A^(n-1);
end
clear n

F3.F3 = [F3.F3b1 zeros(size(F3.F3b1,1),size(F3.F3b2,2)) zeros(size(F3.F3b1,1),size(F3.F3d,2));
      zeros(size(F3.F3b2,1),size(F3.F3b1,2)) F3.F3b2 zeros(size(F3.F3b2,1),size(F3.F3d,2))
      zeros(size(F3.F3d,1),size(F3.F3b1,2)) zeros(size(F3.F3d,1),size(F3.F3b2,2)) F3.F3d];
  
%%
F3.F3new = [F3.F3;
            zeros(2*dim.Np,3)];

%% Constructing W matrices for optimization
W1.W1b1 = [dim.Wb1*ones(1,dim.Np-1) 0];
W1.W1b2 = [dim.Wb2*ones(1,dim.Np-1) 0];
W1.W1d = [dim.Wd*ones(1,dim.Np-1) 0];
W1.W1 = [ W1.W1b1 W1.W1b2 W1.W1d ];

%% W2 matrices again using submatrices
% Bij W2 moet eigenlijk ook de rij van de Np^e entry 0 zijn, maar omdat
% W1*W2, doen we dit nu even niet.
W2.W2b1 = zeros(dim.Np,3*dim.Np);
W2.W2b2 = zeros(dim.Np,3*dim.Np);
W2.W2d  = zeros(dim.Np,9*dim.Np);

for nr = 1:dim.Np
    for nc = 1:3*dim.Np
        if nr == nc
            if nc <= dim.Np
                W2.W2b1(nr,nc) = 1;
                W2.W2b2(nr,nc) = 1;
            end
        end
        
        if  nr == nc+1
                W2.W2b1(nr,nc) = -1;
                W2.W2b2(nr,nc) = -1;
        end
    end
   
    for nc = 1:9*dim.Np
        if nc == 1+(nr-1)*4
            W2.W2d(nr,nc:nc+3) = 1;
        end
        
    end
    
    for nc = 1:9*dim.Np-1
        if nc == 1+(nr-1)*4
            W2.W2d(nr+1,nc:nc+3) = -1;
        end
    end
    
end

W2.W2d = W2.W2d(1:end-1,:);

W2.W2 = [W2.W2b1 zeros(size(W2.W2b1,1),size(W2.W2b2,2)) zeros(size(W2.W2b1,1),size(W2.W2d,2)); ...
         zeros(size(W2.W2b2,1),size(W2.W2b1,2)) W2.W2b2 zeros(size(W2.W2b2,1),size(W2.W2d,2)); ...
         zeros(size(W2.W2d,1),size(W2.W2b1,2)) zeros(size(W2.W2d,1),size(W2.W2b2,2)) W2.W2d];

W2.W2new = [W2.W2 zeros(size(W2.W2,1),dim.Np)];
     
%% W3 matrices
W3.W3b1 = [zeros(1,dim.Np-1) -dim.We];
W3.W3b2 = [zeros(1,dim.Np-1) -dim.We];
W3.W3d  = [zeros(1,dim.Np-1) -dim.Wfuel]; 
W3.W3 = [W3.W3b1 W3.W3b2 W3.W3d];

% W4 matrices
W4.W4b1 = dim.We;
W4.W4b2 = dim.We;
W4.W4d  = dim.Wfuel;
W4.W4 = [W4.W4b1 W4.W4b2 W4.W4d];

%% W5 matrices
Ce = zeros(1,dim.Np);
for k = 1:dim.Np
    Ce(k) = 50 + 50*sin(pi*dim.Ts*k / 12);
end
clear k

save('Ce.mat','Ce')

W5.W5b1 = [zeros(1,dim.Np) -Ce(1,1:dim.Np-1) 0 zeros(1,dim.Np)];
W5.W5b2 = [zeros(1,dim.Np) -Ce(1,1:dim.Np-1) 0 zeros(1,dim.Np)];
W5.W5d  = [zeros(1,4*dim.Np) -Ce(1,1:dim.Np-1) 0 zeros(1,4*dim.Np)];

W5.W5 = [W5.W5b1 W5.W5b2 W5.W5d];

%% S matrices OLD
S1.S1b1 = W3.W3b1*M1.M1_b1+W5.W5b1;
S1.S1b2 = W3.W3b2*M1.M1_b2+W5.W5b2;
S1.S1d  = W3.W3d*M1.M1_d+W5.W5d;

S1.S1 = W3.W3*M1.M1+W5.W5;

S1.S1b1new = [W3.W3b1*M1.M1_b1 Ce(1:dim.Np-1) 0];
S1.S1b2new = [W3.W3b2*M1.M1_b2 Ce(1:dim.Np-1) 0];
S1.S1dnew  = [W3.W3d*M1.M1_d+W5.W5d Ce(1:dim.Np-1) 0];

S1.S1new = [W3.W3*M1.M1+W5.W5 Ce(1:dim.Np-1) 0];

S2.S2b1 = W3.W3b1*M2.M2_b1*W4.W4b1;
S2.S2b2 = W3.W3b2*M2.M2_b2*W4.W4b2;
S2.S2d  = W3.W3d*M2.M2_d*W4.W4d;
S2.S2   = W3.W3*M2.M2+W4.W4;

%% Saving matrices
save('M1.mat','M1'); save('M2.mat','M2'); save('M3.mat','M3');
save('F1.mat','F1'); save('F2.mat','F2'); save('F3.mat','F3');
save('W1.mat','W1'); save('W2.mat','W2'); save('W3.mat','W3'); save('W4.mat','W4'); save('W5.mat','W5');
save('S1.mat','S1'); save('S2.mat','S2');

%% Optimizing entire system
xb1(1) = parB.x0;
xb2(1) = parB.x0;
xd(1)  = parD.x0;

x(:,1) = [xb1(1); xb2(1); xd(1)];
k = 1;

% Minimize 
%       W1 H + S1 V + S2 x 
% Subject to
%            F1 V <= F2 + F3*x(k)
%       -H - W2 V <= 0
%       -H + W2 V <= 0

% names = {'H'; 'V'; 'X'};

% Cost function to minimize
model.obj = [W1.W1 S1.S1];
model.modelsense = 'min';
% model.varnames = names;
model.vtype = [repmat('C',3*dim.Np,1); ...
               repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',4*dim.Np,1); repmat('C',dim.Np,1); repmat('S',4*dim.Np,1)];

% Constraints
model.A = sparse([zeros(size(F1.F1,1),size(W1.W1,2)) F1.F1; ...
                  -eye(size(W2.W2,1)) -W2.W2; ...
                  -eye(size(W2.W2,1)) W2.W2 ]);
model.rhs = [F2.F2+F3.F3*x(:,k); zeros(size(W2.W2,1),1); zeros(size(W2.W2,1),1)];
model.sense = repmat('<',size(F2.F2,1)+2*size(W2.W2,1),1);

% Gurobi Solve
gurobi_write(model, 'mip1.lp');
params.outputflag = 0;
result = gurobi(model, params);
disp(result);

% Gurobi Solve 2
% for v=1:length(names)
%     fprintf('%s %d\n', names{v}, result.x(v));
% end
% fprintf('Obj: %e\n', result.objval);
% end
% result = gurobi(model, params);
% disp(result);

%% Optimizing battery 1
% Minimize
%       W1b1 H + S1b1 V + S2b1 x
% Subject to
%            F1b1 V <= F2b1 + F3b1*xb1(k) 
%       -H - W2b1 V <= 0
%       -H + W2b1 V <= 0

% names = {'H'; 'V'; 'X'};

k = 1;
xb1(1) = 10;

% Cost function to minimize
modelb1.obj = [W1.W1b1 S1.S1b1];
modelb1.modelsense = 'min';
% model.varnames = names;
modelb1.vtype = [repmat('C',dim.Np,1); ...
                 repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1);];

% Constraints
modelb1.A = sparse([zeros(size(F1.F1b1,1),size(W1.W1b1,2)) F1.F1b1; ...
                    -eye(size(W2.W2b1,1)) -W2.W2b1; ...
                    -eye(size(W2.W2b1,1)) W2.W2b1]);
modelb1.rhs = [F2.F2b1+F3.F3b1*xb1(k); zeros(size(W2.W2b1,1),1); zeros(size(W2.W2b1,1),1)];
modelb1.sense = repmat('<',size(F2.F2b1,1)+2*size(W2.W2b1,1),1);

% Gurobi Solve
gurobi_write(modelb1, 'mip1.lp');
params.outputflag = 0;
result = gurobi(modelb1, params);
disp(result);

disp(result.x);

%% Optimizing the diesel generator
xd(1) = 50;

% Minimize
%       W1b1 H + S1b1 V + S2b1 x
% Subject to
%       F1b1 V <= F2b1 + F3d*xd(k)
%       -H - W2b1 V <= 0
%       -H + W2b1 V <= 0

% names = {'H'; 'V'; 'X'};

% Cost function to minimize
modeld.obj = [W1.W1d S1.S1d];
modeld.modelsense = 'min';
% model.varnames = names;
modeld.vtype = [repmat('C',dim.Np,1); ...
                 repmat('B',4*dim.Np,1); repmat('C',dim.Np,1); repmat('S',4*dim.Np,1)];

% Constraints
modeld.A = sparse([zeros(size(F1.F1d,1),size(W1.W1d,2)) F1.F1d; ...
                    -eye(size(W2.W2d,1)) -W2.W2d; ...
                    -eye(size(W2.W2d,1)) W2.W2d]);
modeld.rhs = [F2.F2d+F3.F3d*xd(k); zeros(size(W2.W2d,1),1); zeros(size(W2.W2d,1),1)];
modeld.sense = repmat('<',size(F2.F2d,1)+2*size(W2.W2d,1),1);

% Gurobi Solve
gurobi_write(modeld, 'mip1.lp');
params.outputflag = 0;
result = gurobi(modeld, params);
disp(result);

%% Optimizing the diesel generator without absolute values
xd(1) = parD.x0;

% Minimize
%       S1d V
% Subject to
%       F1d V <= F2d + F3d*xd(k)

% names = {'V'};

% Cost function to minimize
modeld1.obj = [S1.S1d];
modeld1.modelsense = 'min';
% model.varnames = names;
modeld1.vtype = [repmat('B',4*dim.Np,1); repmat('C',dim.Np,1); repmat('S',4*dim.Np,1)];

% Constraints
modeld1.A = sparse([F1.F1d]);
modeld1.rhs = [F2.F2d+F3.F3d*xd(k)];
modeld1.sense = repmat('<',size(F2.F2d,1),1);

% Gurobi Solve
gurobi_write(modeld1, 'mip1.lp');
params.outputflag = 0;
result = gurobi(modeld1, params);
disp(result);

%% Optimizing entire system NEW
xb1(1) = parB.x0;
xb2(1) = parB.x0;
xd(1)  = parD.x0;

x(:,1) = [xb1(1); xb2(1); xd(1)];
k = 1;

% Minimize 
%       W1 H + S1 V
% Subject to
%            F1 V <= F2 + F3*x(k)
%       -H - W2 V <= 0
%       -H + W2 V <= 0

% names = {'H'; 'V'; 'X'};

% Cost function to minimize
model.obj = [W1.W1 S1.S1new];
model.modelsense = 'min';
% model.varnames = names;
model.vtype = [repmat('C',3*dim.Np,1); ...
               repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',4*dim.Np,1); repmat('C',dim.Np,1); repmat('S',4*dim.Np,1); repmat('C',dim.Np,1)];

% Constraints
model.A = sparse([zeros(size(F1.F1new,1),size(W1.W1,2)) F1.F1new; ...
                  -eye(size(W2.W2new,1)) -W2.W2new; ...
                  -eye(size(W2.W2new,1)) W2.W2new ]);
model.rhs = [F2.F2new+F3.F3new*x(:,k); zeros(size(W2.W2new,1),1); zeros(size(W2.W2new,1),1)];
model.sense = repmat('<',size(F2.F2new,1)+2*size(W2.W2new,1),1);

% Gurobi Solve
gurobi_write(model, 'mip1.lp');
params.outputflag = 0;
result = gurobi(model, params);
disp(result);

% Gurobi Solve 2
% for v=1:length(names)
%     fprintf('%s %d\n', names{v}, result.x(v));
% end
% fprintf('Obj: %e\n', result.objval);
% end
% result = gurobi(model, params);
% disp(result);


