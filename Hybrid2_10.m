%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.10
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;

%% Loading data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;
load M1.mat; load M2.mat; load M3.mat; load F1.mat; load F2.mat; load F3.mat;
load W1.mat; load W2.mat; load W3.mat; load W4.mat; load W5.mat; load S1.mat; load S2.mat;
load Ce.mat;

%% Defining Pload (needed for the cost)
for k = 1:100
    if k <= 20
        Pload(k) = 0;
    elseif k >= 21 && k <= 50
        Pload(k) = 30+2*k;
    elseif k >= 51
        Pload(k) = 45;
    end
end

parB.x0 = 10;
parD.x0 = 50;

xb1(1) = parB.x0;
xb2(1) = parB.x0;
xd(1)  = parD.x0;


%% MPC loop for battery 1

for  k = 1:150
    for i = 1:2
    Ce(k) = 50 + 50*sin(pi*dim.Ts*k / 12);
    
    if k = setdiff(1:150,60*i)  % if it is not 12AM or 12PM
    % Minimize
    %       W1b1 H + S1b1 V + S2b1 x
    % Subject to
    %       F1b1 V - F3b1 x <= F2b1 
    %       -H - W2b1 V <= 0
    %       -H + W2b1 V <= 0

    % names = {'H'; 'V'; 'X'};
   
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
    
    x_opti(:,k) = result.x;
    
    disp(modelb1.obj(2*dim.Np+1))
    
    % Battery cost function at time step k
    J(k) = result.objval + S2.S2b1*xb1(k) + Ce(k)*Pload(k) + W3.W3b1*M3.M3_b1;

    % Update equation battery 1
    xb1(k+1) = MLDB1.A*xb1(k) + MLDB1.B1*result.x(2*dim.Np+1) + MLDB1.B3*result.x(3*dim.Np+1);
    end
    
     if k = 60*i;
    % New constraints
    MLDB1.E1.2 = [MLDB1.E1; -1];
    MLDB1.E2.2 = [MLDB1.E2; 0];
    MLDB1.E3.2 = [MLDB1.E3; 0];
    MLDB1.E4.2 = [MLDB1.E4; 0];
    MLDB1.g5.2 = [MLDB1.g5; -0.2*parB.x_up(1,1)];
    
     end
    end
end

