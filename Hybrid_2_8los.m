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
modelb1.obj = [W1.W1b1 S1.S1b1new];
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