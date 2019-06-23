%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.9
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

%% Loading data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDDJordan.mat; load parB.mat; load parD.mat;
load M1.mat; load M2.mat; load M3.mat; load F1.mat; load F2.mat; load F3.mat;
load W1.mat; load W2.mat; load W3.mat; load W4.mat; load W5.mat; load S1.mat; load S2.mat;
load Ce.mat;

%%
dim.Tend = 360;
xb1 = zeros(1,dim.Tend);
xb2 = zeros(1,dim.Tend);
xd  = zeros(1,dim.Tend);
xb1(1) = parB.x0;
xb2(1) = parB.x0;
xd(1)  = parD.x0;
x   = [xb1; xb2; xd];
x(:,1) = [xb1(1); xb2(1); xd(1)];

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

% %% MPC loop for battery 1
% 
% for  k = 1:50
%     
%     Ce(k) = 50 + 50*sin(pi*dim.Ts*k / 12);
%     
%     % Minimize
%     %       W1b1 H + S1b1 V + S2b1 x
%     % Subject to
%     %       F1b1 V - F3b1 x <= F2b1 
%     %       -H - W2b1 V <= 0
%     %       -H + W2b1 V <= 0
% 
%     % names = {'H'; 'V'; 'X'};
% 
%     % Cost function to minimize
%     modelb1.obj = [W1.W1b1 S1.S1b1];
%     modelb1.modelsense = 'min';
%     % model.varnames = names;
%     modelb1.vtype = [repmat('C',dim.Np,1); ...
%                      repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1);];
% 
%     % Constraints
%     modelb1.A = sparse([zeros(size(F1.F1b1,1),size(W1.W1b1,2)) F1.F1b1; ...
%                         -eye(size(W2.W2b1,1)) -W2.W2b1; ...
%                         -eye(size(W2.W2b1,1)) W2.W2b1]);
%     modelb1.rhs = [F2.F2b1+F3.F3b1*xb1(k); zeros(size(W2.W2b1,1),1); zeros(size(W2.W2b1,1),1)];
%     modelb1.sense = repmat('<',size(F2.F2b1,1)+2*size(W2.W2b1,1),1);
% 
%     % Gurobi Solve
%     gurobi_write(modelb1, 'mip1.lp');
%     params.outputflag = 0;
%     result = gurobi(modelb1, params);
%     disp(result);
%     
%     x_opti(:,k) = result.x;
%     
%     disp(modelb1.obj(2*dim.Np+1))
%     
%     % Battery cost function at time step k
%     J(k) = result.objval + S2.S2b1*x(:,k) + W3.W3b1*M3.M3b1;
% 
%     % Update equation battery 1
%     xb1(k+1) = MLDB1.A*xb1(k) + MLDB1.B1*result.x(2*dim.Np+1) + MLDB1.B3*result.x(3*dim.Np+1);
%     
% end

%%

for k = 1:dim.Tend
    % Minimize 
    %       W1 H + S1new Vnew
    % Subject to
    %            F1new Vnew <= F2new + F3new*x(k)
    %       -H - W2new Vnew <= 0
    %       -H + W2new Vnew <= 0

    % names = {'H'; 'V'};

    % Cost function to minimize
    model.obj = [W1.W1 S1.S1new];
    model.modelsense = 'min';
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
    
    % Optimized vector
    x_opti(:,k) = result.x;
    
    % Cost function at time step k
    J(k) = result.objval + S2.S2*x(:,k) + W3.W3*M3.M3;

    % Update equation
    xb1(:,k+1) = MLDB1.A*xb1(:,k) + MLDB1.B1*result.x(dim.Np+1) + MLDB1.B3*result.x(2*dim.Np+1);
    xb2(:,k+1) = MLDB2.A*xb2(:,k) + MLDB2.B1*result.x(dim.Np+1) + MLDB2.B3*result.x(2*dim.Np+1);
    xd(:,k+1)  = MLDD.A*xd(:,k) + MLDD.B2*result.x(6*dim.Np+1:6*dim.Np+4) + MLDD.B3*result.x(11*dim.Np+1:11*dim.Np+4) + MLDD.B4; 
    
    x(:,k+1) = [xb1(:,k+1);
                xb2(:,k+1)
                xd(:,k+1)];
    
end

%%
figure; hold on;
% plot(xb1)
% plot(xb2)
plot(xd(1,:))
hold off;

figure; hold on;
plot(x_opti(10*dim.Np+1,:))

