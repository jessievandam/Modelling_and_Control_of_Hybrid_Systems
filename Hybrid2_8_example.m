%% Joost
model.obj = -sum(M);
model.A = sparse(F_1);
model.rhs = F_2 + F_3 * x(1);
model.sense = repmat('<',size(F_1,1),1);
model.vtype = [repmat('B',5*dim.Np,1); repmat('C', 5*dim.Np,1)];%['B'; 'C'; 'C'];
model.modelsense = 'min';

gurobi_write(model, 'mip1.lp');

params.outputflag = 0;

%% Example MIP1
% This example formulates and solves the following simple MIP model:
%  maximize
%        x +   y + 2 z
%  subject to
%        x + 2 y + 3 z <= 4
%        x +   y       >= 1
%        x, y, z binary

names = {'x'; 'y'; 'z'};
model.A = sparse([1 2 3; 1 1 0]);
model.obj = [1 1 2];
model.rhs = [4; 1];
model.sense = '<>';
model.vtype = 'B';
model.modelsense = 'max';
model.varnames = names;

gurobi_write(model, 'mip1.lp');

params.outputflag = 0;

result = gurobi(model, params);

disp(result);

for v=1:length(names)
    fprintf('%s %d\n', names{v}, result.x(v));
end

fprintf('Obj: %e\n', result.objval);
% end
result = gurobi(model, params);

disp(result);

%% Optimizing Info
% The Gurobi MATLAB setup script, gurobi_setup.m, can be found in the 
% <installdir>/matlab directory of your Gurobi installation (the default 
% <installdir> for Gurobi 8.1.1 is /Library/gurobi811/mac64 for Mac). 
cd /Library/gurobi811/mac64/matlab;
gurobi_setup;

% mip1 - builds a trivial MIP model, solves it, and prints the solution