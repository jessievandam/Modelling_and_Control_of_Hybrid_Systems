%% SC4160 HYBRID
clear all; close all; clc;
% Step 2.3: Diesel generator

% Region 1
syms a1 b1 ud1
fun11 = (ud1^2 + 4 - a1 - b1*ud1).^2;
fun12 = (4*ud1 - a1 - b1*ud1).^2
int11 = int(fun11,ud1,0,2);
int12 = int(fun12,ud1,2,5);
g1 = matlabFunction(int11+int12);

pdiff1a = diff(g1,a1);
pdiff1b = diff(g1,b1);

% Region 2
syms a2 b2 ud2
fun2 = (-9.44*ud2^3 + 166.06*ud2^2 -948.22*ud2 + 1790.28 - a2 - b2*ud2).^2;
int2 = int(fun2,ud2,5,6.5);
g2 = matlabFunction(int2);

pdiff2a = diff(g2,a2);
pdiff2b = diff(g2,b2);

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

sola3 = sol3.a3;
solb3 = sol3.b3;

% Region 4
syms a4 b4 ud4
fun4 = (4.01 * (ud4-10.47).^2 + 17.79 - a4 - b4*ud4).^2;
int4 = int(fun4,ud4,11,15);
g4 = matlabFunction(int4);

pdiff4a = diff(g4,a4);
pdiff4b = diff(g4,b4);

sol4 = solve(pdiff4a,pdiff4b);

sola4 = sol4.a4;
solb4 = sol4.b4;






