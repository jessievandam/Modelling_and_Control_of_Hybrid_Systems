%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.8
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

%% Loading previously defined data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;

%% Constructing M matrices for update equation MILP
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

M2.M2 = [M2.M2_b1; M2.M2_b2; M2.M2_d];

% M1.M1 matrix for the diesel generator
M1.M1_d_delta = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B2,2));
M1.M1_d_u = zeros(dim.Np,dim.Np);
% TO DO: NOW M1.M1_d_u IS A ZEROS MATRIX BUT MAKE IT GENERAL!
M1.M1_d_zd = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B3,2));

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1.M1_d_delta(1+(np2-1)*size(MLDD.A,1),1+(np1-1)*size(MLDD.B2,2):np1*size(MLDD.B2,2))  = MLDD.A^(np2-1)*MLDD.B2;
    M1.M1_d_u(1+(np2-1)*size(MLDD.A,1),1+(np1-1):np1*size(MLDD.B1,2))      = MLDD.A^(np2-1)*MLDD.B1;
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
M1.M1_b1_      = zeros(dim.Np,3*dim.Np);

M1.M1_b2_delta = zeros(dim.Np,dim.Np);
M1.M1_b2_u     = zeros(dim.Np,dim.Np);
M1.M1_b2_zd    = zeros(dim.Np,dim.Np);
M1.M1_b2       = zeros(dim.Np,3*dim.Np);

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1.M1_b1_delta(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B2,2):np1*size(MLDB1.B2,2))  = MLDB1.A^(np2-1)*MLDB1.B2;
    M1.M1_b1_u(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B3,2):np1*size(MLDB1.B3,2))      = MLDB1.A^(np2-1)*MLDB1.B3;
    M1.M1_b1_zd(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B1,2):np1*size(MLDB1.B1,2))     = MLDB1.A^(np2-1)*MLDB1.B1;
    
    M1.M1_b2_delta(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B2,2):np1*size(MLDB2.B2,2))  = MLDB2.A^(np2-1)*MLDB2.B2;
    M1.M1_b2_u(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B3,2):np1*size(MLDB2.B3,2))      = MLDB2.A^(np2-1)*MLDB2.B3;
    M1.M1_b2_zd(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B1,2):np1*size(MLDB2.B1,2))     = MLDB2.A^(np2-1)*MLDB2.B1;
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

%% Constructing F1.F1 matrix for MILP constraint equation
% NOTE: maybe put all submatrices in a struct
F1.F1_b1_delta = zeros(dim.Np*size(MLDB1.E3,1),dim.Np*size(MLDB1.E3,2));
F1.F1_b1_u = zeros(dim.Np*size(MLDB1.E2,1),dim.Np*size(MLDB1.E2,2));
F1.F1_b1_zd = zeros(dim.Np*size(MLDB1.E4,1),dim.Np*size(MLDB1.E4,2));

F1.F1_b2_delta = zeros(dim.Np*size(MLDB2.E3,1),dim.Np*size(MLDB2.E3,2));
F1.F1_b2_u = zeros(dim.Np*size(MLDB2.E2,1),dim.Np*size(MLDB2.E2,2));
F1.F1_b2_zd = zeros(dim.Np*size(MLDB2.E4,1),dim.Np*size(MLDB2.E4,2));

F1.F1_d_delta = zeros(dim.Np*size(MLDD.E3,1),dim.Np*size(MLDD.E3,2));
F1.F1_d_u = zeros(dim.Np*size(MLDD.E2,1),dim.Np*size(MLDD.E2,2));
F1.F1_d_zd = zeros(dim.Np*size(MLDD.E3,1),dim.Np*size(MLDD.E4,2));

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
        if np1 == np2
        % For battery 1
        F1.F1_b1_delta(1+(np2-1)*size(MLDB1.E3,1):np2*size(MLDB1.E3,1),1+(np1-1)*size(MLDB1.E3,2):np1*size(MLDB1.E3,2))    = MLDB1.E3;
        F1.F1_b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2))        = MLDB1.E2;
        F1.F1_b1_zd(1+(np2-1)*size(MLDB1.E4,1):np2*size(MLDB1.E4,1),1+(np1-1)*size(MLDB1.E4,2):np1*size(MLDB1.E4,2))       = MLDB1.E4;
        
        % For battery 2
        F1.F1_b2_delta(1+(np2-1)*size(MLDB2.E3,1):np2*size(MLDB2.E3,1),1+(np1-1)*size(MLDB2.E3,2):np1*size(MLDB2.E3,2))    = MLDB2.E3;
        F1.F1_b2_u(1+(np2-1)*size(MLDB2.E2,1):np2*size(MLDB2.E2,1),1+(np1-1)*size(MLDB2.E2,2):np1*size(MLDB2.E2,2))        = MLDB2.E2;
        F1.F1_b2_zd(1+(np2-1)*size(MLDB2.E4,1):np2*size(MLDB2.E4,1),1+(np1-1)*size(MLDB2.E4,2):np1*size(MLDB2.E4,2))       = MLDB2.E4;
        
        % For the diesel generator
        F1.F1_d_delta(1+(np2-1)*size(MLDD.E3,1):np2*size(MLDD.E3,1),1+(np1-1)*size(MLDD.E3,2):np1*size(MLDD.E3,2)) = MLDD.E3;
        F1.F1_d_u(1+(np2-1)*size(MLDD.E2,1):np2*size(MLDD.E2,1),1+(np1-1)*size(MLDD.E2,2):np1*size(MLDD.E2,2))     = MLDD.E2;
        F1.F1_d_zd(1+(np2-1)*size(MLDD.E4,1):np2*size(MLDD.E4,1),1+(np1-1)*size(MLDD.E4,2):np1*size(MLDD.E4,2))    = MLDD.E4;
        end
        
        if np2 > np1 % to fill only below the block diagonal
        % NOTE: algemener maken voor F1.F1_bi_delta?
        % For battery 1
        F1.F1_b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2))  = MLDB1.E1*MLDB1.A^(np2-2)*MLDB1.B1;
        F1.F1_b1_zd(1+(np2-1)*size(MLDB1.E4,1):np2*size(MLDB1.E4,1),1+(np1-1)*size(MLDB1.E4,2):np1*size(MLDB1.E4,2)) = MLDB1.E1*MLDB1.A^(np2-2)*MLDB1.B3;
        
        % For battery 2
        F1.F1_b2_u(1+(np2-1)*size(MLDB2.E2,1):np2*size(MLDB2.E2,1),1+(np1-1)*size(MLDB2.E2,2):np1*size(MLDB2.E2,2))  = MLDB2.E1*MLDB2.A^(np2-2)*MLDB2.B1;
        F1.F1_b2_zd(1+(np2-1)*size(MLDB2.E4,1):np2*size(MLDB2.E4,1),1+(np1-1)*size(MLDB2.E4,2):np1*size(MLDB2.E4,2)) = MLDB2.E1*MLDB2.A^(np2-2)*MLDB2.B3;
        
        % For the diesel generator
        F1.F1_d_u(1+(np2-1)*size(MLDD.E3,1):np2*size(MLDD.E3,1),1+(np1-1)*size(MLDD.E3,2):np1*size(MLDD.E3,2)) = MLDD.E1*MLDD.A^(np2-2)*MLDD.B2;
        F1.F1_d_zd(1+(np2-1)*size(MLDD.E4,1):np2*size(MLDD.E4,1),1+(np1-1)*size(MLDD.E4,2):np1*size(MLDD.E4,2)) = MLDD.E1*MLDD.A^(np2-2)*MLDD.B3;
        end
    end
end
clear np1 np2

% Concatenating all submatrices
F1.F1_b1 = [F1.F1_b1_delta F1.F1_b1_u F1.F1_b1_zd];
F1.F1_b2 = [F1.F1_b2_delta F1.F1_b2_u F1.F1_b2_zd];
F1.F1_d = [F1.F1_d_delta F1.F1_d_u F1.F1_d_zd];
F1.F1 = [F1.F1_b1 zeros(size(F1.F1_b1,1),size(F1.F1_b2,2)) zeros(size(F1.F1_b1,1),size(F1.F1_d,2));
      zeros(size(F1.F1_b2,1),size(F1.F1_b1,2)) F1.F1_b2 zeros(size(F1.F1_b2,1),size(F1.F1_d,2))
      zeros(size(F1.F1_d,1),size(F1.F1_b1,2)) zeros(size(F1.F1_d,1),size(F1.F1_b2,2)) F1.F1_d];

%% Constructing F2.F2 matrix for MILP constraint equation
F2.F2_b1   = zeros(dim.Np*size(MLDB1.g5,1),size(MLDB1.g5,2));
F2.F2_b2   = zeros(dim.Np*size(MLDB2.g5,1),size(MLDB2.g5,2));
F2.F2_d_1  = zeros(dim.Np*size(MLDD.g5,1),size(MLDD.g5,2));
F2.F2_d_2  = zeros(dim.Np*size(MLDD.g5,1),size(MLDD.g5,2));

for n = 1:dim.Np
    F2.F2_b1(1+(n-1)*size(MLDB1.g5,1):n*size(MLDB1.g5,1),1:size(MLDB1.g5,2)) = MLDB1.g5;
    F2.F2_b2(1+(n-1)*size(MLDB2.g5,1):n*size(MLDB2.g5,1),1:size(MLDB2.g5,2)) = MLDB2.g5;
    F2.F2_d_1(1+(n-1)*size(MLDD.g5,1):n*size(MLDD.g5,1),1:size(MLDD.g5,2)) = MLDD.g5;
    
    if n == 2
        F2.F2_d_2(1+(n-1)*size(MLDD.E1):n*size(MLDD.E1,1),1:size(MLDD.g5,2)) = MLDD.E1*MLDD.B4;
    end
   
    if n > 2
        F2.F2_d_2(1+(n-1)*size(MLDD.E1):n*size(MLDD.E1,1),1:size(MLDD.g5,2)) = F2.F2_d_2(1+(n-2)*size(MLDD.E1,1):(n-1)*size(MLDD.E1,1)) + MLDD.E1*MLDD.B4*(MLDD.A)^(n-2);
    end
    
end
clear n

% Concatenating submatrices into complete F2.F2 matrix
F2.F2_d = F2.F2_d_1 + F2.F2_d_2;
F2.F2 = [F2.F2_b1; F2.F2_b2; F2.F2_d];

%% Constructing F3.F3 matrix for MILP constraint equation
F3.F3_b1 = zeros(dim.Np*size(MLDB1.E1,1),size(MLDB1.E1,2));
F3.F3_b2 = zeros(dim.Np*size(MLDB2.E1,1),size(MLDB2.E1,2));
F3.F3_d  = zeros(dim.Np*size(MLDD.E1,1),size(MLDD.E1,2));

for n = 1:dim.Np
    F3.F3_b1(1+(n-1)*size(MLDB1.E1,1):n*size(MLDB1.E1,1),1:size(MLDB1.E1,2)) = -MLDB1.E1*MLDB1.A^(n-1);
    F3.F3_b2(1+(n-1)*size(MLDB2.E1,1):n*size(MLDB2.E1,1),1:size(MLDB2.E1,2)) = -MLDB2.E1*MLDB2.A^(n-1);
    F3.F3_d(1+(n-1)*size(MLDD.E1,1):n*size(MLDD.E1,1),1:size(MLDD.E1,2)) = -MLDD.E1*MLDD.A^(n-1);
end
clear n

F3.F3 = [F3.F3_b1 zeros(size(F3.F3_b1,1),size(F3.F3_b2,2)) zeros(size(F3.F3_b1,1),size(F3.F3_d,2));
      zeros(size(F3.F3_b2,1),size(F3.F3_b1,2)) F3.F3_b2 zeros(size(F3.F3_b2,1),size(F3.F3_d,2))
      zeros(size(F3.F3_d,1),size(F3.F3_b1,2)) zeros(size(F3.F3_d,1),size(F3.F3_b2,2)) F3.F3_d];

%% Constructing W matrices for optimization
W1.W1b1 = dim.Wb1*ones(1,dim.Np);
W1.W1b2 = dim.Wb2*ones(1,dim.Np);
W1.W1d = dim.Wd*ones(1,dim.Np);
W1.W1 = [ W1.W1b1 W1.W1b2 W1.W1d ];

%% W2.W2 matrix
W2.W2_deltab1 = zeros(6*dim.Np,dim.Np);
W2.W2_ub1 = zeros(6*dim.Np,dim.Np);
W2.W2_zb1 = zeros(6*dim.Np,dim.Np);

W2.W2_deltab2 = zeros(6*dim.Np,dim.Np);
W2.W2_ub2 = zeros(6*dim.Np,dim.Np);
W2.W2_zb2 = zeros(6*dim.Np,dim.Np);

W2.W2_deltad = zeros(6*dim.Np,4*dim.Np);
W2.W2_ud = zeros(6*dim.Np,dim.Np);
W2.W2_zdd = zeros(6*dim.Np,4*dim.Np);

for nr = 1:6*dim.Np %rows
    for nc = 1:dim.Np %columns
        if nr == nc
            W2.W2_deltab1(nr,nc) = 1;
        end
        
        if nr > (dim.Np) && nr < (2*dim.Np+1)
            if nr == nc+dim.Np
                W2.W2_deltab2(nr,nc) = 1;
            elseif nr == nc+dim.Np+1
                W2.W2_deltab2(nr,nc) = -1;
            end
        end
                
    end
    
    for nc = 1:4*dim.Np
        if nr == nc + 2*dim.Np
            W2.W2_deltad(nr,nc) = 1;
        elseif nr == nc + 3*dim.Np + 1
            W2.W2_deltad(nr,nc) = -1;
        end
    end
end
clear nr nc

W2.W2 = [W2.W2_deltab1 W2.W2_ub1 W2.W2_zb1 W2.W2_deltab2 W2.W2_ub2 W2.W2_zb2 W2.W2_deltad W2.W2_ud W2.W2_zdd];

% W3 matrices
W3.W3b1 = ;
W3.W3b2 = ;
W3.W3d  =; 
W3 = [zeros(1,dim.Np-1) -dim.We zeros(1,dim.Np-1) -dim.We zeros(1,dim.Np-1) -dim.We];
W4 = [dim.We dim.We dim.Wfuel];

% W5 matrix
Ce = zeros(1,dim.Np);
for k = 1:dim.Np
    Ce(k) = 50 + 50*sin(pi*dim.Ts*k / 12);
end
clear k

W5 = [zeros(1,dim.Np) -Ce(1,1:dim.Np) zeros(1,dim.Np) zeros(1,dim.Np) -Ce(1,1:dim.Np) zeros(1,dim.Np) zeros(1,4*dim.Np) -Ce(1,1:dim.Np) zeros(1,4*dim.Np)];

%% S matrices
S1 = W3*M1.M1+W5;
S2 = W3*M2.M2+W4;

%% Optimizing
% Minimize
%       W1 H + S1 V + S2 x
% Subject to
%       F1 V - F3 x <= F2 
%       -H - W2 V <= 0
%       -H + W2 V <= 0

names = {'H'; 'V'; 'X'};

% Cost function to minimize
model.obj = [W1.W1 S1 S2];
model.modelsense = 'min';
model.varnames = names;
%model.vtype = 'C';             % B = binary, C = continuous

% Constraints
model.A = sparse([0 F1.F1 -F3.F3; -1 -W2.W2 0; -1 W2.W2 0]);
model.rhs = [F2.F2; 0; 0];
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



