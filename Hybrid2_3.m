%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.3
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;

% Region 1
syms a1 b1 ud1
fun11 = (ud1^2 + 4 - a1 - b1*ud1).^2;
fun12 = (4*ud1 - a1 - b1*ud1).^2;
int11 = int(fun11,ud1,0,2);
int12 = int(fun12,ud1,2,5);
g1 = matlabFunction(int11+int12);

pdiff1a = diff(g1,a1);
pdiff1b = diff(g1,b1);

sol1 = solve(pdiff1a,pdiff1b);

parD.a1 = double(sol1.a1);
parD.b1 = double(sol1.b1);

% Region 2
syms a2 b2 ud2
fun2 = (-9.44*ud2^3 + 166.06*ud2^2 -948.22*ud2 + 1790.28 - a2 - b2*ud2).^2;
int2 = int(fun2,ud2,5,6.5);
g2 = matlabFunction(int2);

pdiff2a = diff(g2,a2);
pdiff2b = diff(g2,b2);

sol2 = solve(pdiff2a,pdiff2b);

parD.a2 = double(sol2.a2);
parD.b2 = double(sol2.b2);

% Region 3
syms a3 b3 ud3
fun31 = (-9.44*ud3^3 + 166.06*ud3^2 -948.22*ud3 + 1790.28 - a3 - b3*ud3).^2;
fun32 = (-11.78*ud3 + 132.44 - a3 - b3*ud3).^2;
fun33 = (4.01*(ud3-10.47).^2 + 17.79 - a3 - b3*ud3).^2;
int31 = int(fun31,ud3,6.5,7);
int32 = int(fun32,ud3,7,9);
int33 = int(fun33,ud3,9,11);
g3 = matlabFunction(int31 + int32 + int33);

pdiff3a = diff(g3,a3);
pdiff3b = diff(g3,b3);

sol3 = solve(pdiff3a,pdiff3b);

parD.a3 = double(sol3.a3);
parD.b3 = double(sol3.b3);

% Region 4
syms a4 b4 ud4
fun4 = (4.01 * (ud4-10.47).^2 + 17.79 - a4 - b4*ud4).^2;
int4 = int(fun4,ud4,11,15);
g4 = matlabFunction(int4);

pdiff4a = diff(g4,a4);
pdiff4b = diff(g4,b4);

sol4 = solve(pdiff4a,pdiff4b);

parD.a4 = double(sol4.a4);
parD.b4 = double(sol4.b4);

save('parD.mat','parD');

%% Plot real function together with approximation
% real function
ud = 0:0.05:15;

for i = 1:301;
    if i < 41
    funreal(i) = ud(i)^2+4;
    elseif i < 101
    funreal(i) = 4*ud(i);
    elseif i < 141
    funreal(i) = -9.44*ud(i)^3+166.06*ud(i)^2-948.22*ud(i)+1790.28;
    elseif i < 181
    funreal(i) = -11.78*ud(i) + 132.44;
    elseif i > 180
    funreal(i) = 4.01*(ud(i)-10.47)^2+17.79;
    end
end

% approximate function
for i = 1:301;
    if i < 101
    funapprox(i) = parD.a1+parD.b1*ud(i); 
    elseif i < 131
    funapprox(i) = parD.a2+parD.b2*ud(i);     
    elseif i < 221
    funapprox(i) = parD.a3+parD.b3*ud(i);     
    elseif i > 220
    funapprox(i) = parD.a4+parD.b4*ud(i); 
    end
end

figure;
hold on; grid on;
plot(ud,funreal);
plot(ud,funapprox);
xlabel('generated power ud [kW]');
ylabel('consumed fuel of diesel generator [kg/h]');
legend('real fuel consumption', 'approximated fuel consumption');

