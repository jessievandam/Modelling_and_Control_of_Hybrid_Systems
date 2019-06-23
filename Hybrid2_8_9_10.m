%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.8 2.9 and 2.10
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

%% Loading and defining parameters and data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;
load M1.mat; load M2.mat; load M3.mat; load F1.mat; load F2.mat; load F3.mat;
load W1.mat; load W2.mat; load W3.mat; load W4.mat; load W5.mat; load S1.mat; load S2.mat;
load Ce.mat;

parB.x0 = 10;
parD.x0 = 50;
dim.Tend = 120; % Number of timesteps to optimize for

xb1 = zeros(1,dim.Tend);
xb2 = zeros(1,dim.Tend);
xd  = zeros(1,dim.Tend);
xb1(1) = parB.x0;
xb2(1) = parB.x0;
xd(1)  = parD.x0;
x   = [xb1; xb2; xd];
x(:,1) = [xb1(1); xb2(1); xd(1)];

%% Defining Pload
for k = 1:dim.Tend
    if k <= 20
        Pload(k) = 0;
    elseif k >= 21 && k <= 50
        Pload(k) = 30+2*k;
    elseif k >= 51
        Pload(k) = 45;
    end
end

%% 
% Constructing matrices when it's not 12AM 12PM
[W1,W2,W3,F1,F2,F3,S1,S2] = constructMatrices(dim,parB,parD,MLDB1,MLDB2,MLDD,Pload,Ce);

% Constructing matrices when it's 12AM 12PM
MLDB1noon = MLDB1;
MLDB2noon = MLDB2;

MLDB1noon.E1 = [MLDB1.E1; -1];
MLDB1noon.E2 = [MLDB1.E2; 0];
MLDB1noon.E3 = [MLDB1.E3; 0];
MLDB1noon.E4 = [MLDB1.E4; 0];
MLDB1noon.g5 = [MLDB1.g5; -0.2*parB.x_up(1,1)];

MLDB2noon.E1 = [MLDB1.E1; -1];
MLDB2noon.E2 = [MLDB1.E2; 0];
MLDB2noon.E3 = [MLDB1.E3; 0];
MLDB2noon.E4 = [MLDB1.E4; 0];
MLDB2noon.g5 = [MLDB1.g5; -0.2*parB.x_up(1,2)];

[W1noon,W2noon,W3noon,F1noon,F2noon,F3noon,S1noon,S2noon] = constructMatrices(dim,parB,parD,MLDB1noon,MLDB2noon,MLDD,Pload,Ce);


%% Simulating closed-loop behaviour of the system

for k = 1:dim.Tend
    
        if k == 60 | k == 120
            k
            % Minimize 
            %       W1 H + S1new Vnew
            % Subject to
            %            F1new Vnew <= F2new + F3new*x(k)
            %       -H - W2new Vnew <= 0
            %       -H + W2new Vnew <= 0

            % names = {'H'; 'V'};

            % Cost function to minimize
            model.obj = [W1noon.W1 S1noon.S1new];
            model.modelsense = 'min';
            model.vtype = [repmat('C',3*dim.Np,1); ...
                           repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',dim.Np,1); repmat('C',dim.Np,1); repmat('S',dim.Np,1); repmat('B',4*dim.Np,1); repmat('C',dim.Np,1); repmat('S',4*dim.Np,1); repmat('C',dim.Np,1)];

            % Constraints
            model.A = sparse([zeros(size(F1noon.F1new,1),size(W1noon.W1,2)) F1noon.F1new; ...
                              -eye(size(W2noon.W2new,1)) -W2noon.W2new; ...
                              -eye(size(W2noon.W2new,1)) W2noon.W2new ]);
            model.rhs = [F2noon.F2new+F3noon.F3new*x(:,k); zeros(size(W2noon.W2new,1),1); zeros(size(W2noon.W2new,1),1)];
            model.sense = repmat('<',size(F2noon.F2new,1)+2*size(W2noon.W2new,1),1);

            % Gurobi Solve
            gurobi_write(model, 'mip1.lp');
            params.outputflag = 0;
            result = gurobi(model, params);
            disp(result);

        elseif ~(k == 60*i)  % if it is not 12AM or 12PM
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
    
end

%%
figure; hold on;
% plot(xd(1,:))
plot(xb1(1,:))
plot(xd(1,:))
