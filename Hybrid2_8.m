%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.8
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));

%% Loading previously defined data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;

%% Constructing M matrices for update equation MILP
% M2 matrix
M2_b1 = zeros(dim.Np,size(MLDB1.A,2));
M2_b2 = zeros(dim.Np,size(MLDB1.A,2));
M2_d  = zeros(dim.Np,size(MLDD.A,2));
%TO DO: FIX THAT FOR A DIFFERENT LENGTH OF A, THE M2 MATRIX IS FILLED
%CORRECTLY

for np = 1:dim.Np
    M2_b1(np,:) = (MLDB1.A)^np; 
    M2_b2(np,:) = (MLDB2.A)^np;
    M2_d(np,:)  = (MLDD.A)^np;
end

M2 = [M2_b1; M2_b2; M2_d];

% M1 matrix for the diesel generator
M1_d_delta = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B2,2));
M1_d_u = zeros(dim.Np,dim.Np);
% TO DO: NOW M1_d_u IS A ZEROS MATRIX BUT MAKE IT GENERAL!
M1_d_zd = zeros(dim.Np*size(MLDD.A,1),dim.Np*size(MLDD.B3,2));

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1_d_delta(1+(np2-1)*size(MLDD.A,1),1+(np1-1)*size(MLDD.B2,2):np1*size(MLDD.B2,2))  = MLDD.A^(np2-1)*MLDD.B2;
    M1_d_u(1+(np2-1)*size(MLDD.A,1),1+(np1-1):np1*size(MLDD.B1,2))      = MLDD.A^(np2-1)*MLDD.B1;
    M1_d_zd (1+(np2-1)*size(MLDD.A,1),1+(np1-1)*size(MLDD.B3,2):np1*size(MLDD.B3,2))    = MLDD.A^(np2-1)*MLDD.B3;
    end
end

for i = 1:dim.Np
    for j = 1:4*dim.Np
        if j > 4*i
        M1_d_delta(i,j) = 0;
        M1_d_zd(i,j) = 0;
        end
    end
end

M1_d = [M1_d_delta M1_d_u M1_d_zd];

% M1 matrix for the batteries
M1_b1_delta = zeros(dim.Np,dim.Np);
M1_b1_u     = zeros(dim.Np,dim.Np);
M1_b1_zd    = zeros(dim.Np,dim.Np);
M1_b1_      = zeros(dim.Np,3*dim.Np);

M1_b2_delta = zeros(dim.Np,dim.Np);
M1_b2_u     = zeros(dim.Np,dim.Np);
M1_b2_zd    = zeros(dim.Np,dim.Np);
M1_b2       = zeros(dim.Np,3*dim.Np);

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
    M1_b1_delta(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B2,2):np1*size(MLDB1.B2,2))  = MLDB1.A^(np2-1)*MLDB1.B2;
    M1_b1_u(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B3,2):np1*size(MLDB1.B3,2))      = MLDB1.A^(np2-1)*MLDB1.B3;
    M1_b1_zd(1+(np2-1)*size(MLDB1.A,1),1+(np1-1)*size(MLDB1.B1,2):np1*size(MLDB1.B1,2))     = MLDB1.A^(np2-1)*MLDB1.B1;
    
    M1_b2_delta(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B2,2):np1*size(MLDB2.B2,2))  = MLDB2.A^(np2-1)*MLDB2.B2;
    M1_b2_u(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B3,2):np1*size(MLDB2.B3,2))      = MLDB2.A^(np2-1)*MLDB2.B3;
    M1_b2_zd(1+(np2-1)*size(MLDB2.A,1),1+(np1-1)*size(MLDB2.B1,2):np1*size(MLDB2.B1,2))     = MLDB2.A^(np2-1)*MLDB2.B1;
    end
end

% NOTE: MAYBE THIS CAN BE MADE MORE GENERAL (BUT IT'S A LOT OF WORK AND NOT
% THAT NECESSARY)
for i = 1:dim.Np
    for j = 1:dim.Np
        if j > i
        M1_b1_delta(i,j) = 0;
        M1_b1_u(i,j)     = 0;
        M1_b1_zd(i,j)    = 0;
        
        M1_b2_delta(i,j) = 0;
        M1_b2_u(i,j)     = 0;
        M1_b2_zd(i,j)    = 0;
        end
    end
end

M1_b1 = [M1_b1_delta M1_b1_u M1_b1_zd];
M1_b2 = [M1_b2_delta M1_b2_u M1_b2_zd];

% Total M1 matrix
M1 = [M1_b1 zeros(dim.Np,size(M1_b2,2)) zeros(dim.Np,size(M1_d,2)); zeros(dim.Np,size(M1_b1,2)) M1_b2 zeros(dim.Np,size(M1_d,2)); zeros(dim.Np,size(M1_b1,2)) zeros(dim.Np,size(M1_b2,2)) M1_d];

%% Constructing F matrices for MILP constraint equation
F1_b1_delta = zeros(dim.Np*size(MLDB1.E3,1),dim.Np*size(MLDB1.E3,2));
F1_b1_u = zeros(dim.Np*size(MLDB1.E2,1),dim.Np*size(MLDB1.E2,2));
% F1_b1_zd = ;

% F1_b2_delta = ;
% F2_b2_u = ;
% F2_b2_zd = ;
% 
% F2_d_delta = ;
% F2_d_u = ;
% F2_d_zd = ;

for np1 = 1:dim.Np % over columns
    for np2 = 1:dim.Np % over rows
        if np1 == np2
        F1_b1_delta(1+(np2-1)*size(MLDB1.E3,1):np2*size(MLDB1.E3,1),1+(np1-1)*size(MLDB1.E3,2):np1*size(MLDB1.E3,2)) = MLDB1.E3;
        F1_b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2)) = MLDB1.E2;
        end
        
        if np2 > np1 % to fill only below the block diagonal
        F1_b1_u(1+(np2-1)*size(MLDB1.E2,1):np2*size(MLDB1.E2,1),1+(np1-1)*size(MLDB1.E2,2):np1*size(MLDB1.E2,2)) = MLDB1.E1*MLDB1.A^(np2-2)*MLDB1.B1;
        end
    end
end

% 1+(np1-1)*size(MLDB1.E3,2):np1*size(MLDB1.E3,2)