%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.7
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath C:\Users\Miranda\hysdel-2.0.6-MINGW32_NT-5.1-i686
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'))

% Define parameters
parB.eta_c = [0.9 0.95];
parB.eta_d = [0.8 0.77];
parB.x_up  = [48 64];
parB.u_low = [-3 -4];
parB.u_up  = [2 3];
parB.A     = 1;
parB.x0    = 10;

save('parB.mat','parB')

%%
load parD.mat 
parD.x_low = 10;
parD.x_up  = 120;
parD.u_up  = 15;
parD.u_low = 0;
parD.Rf    = 0.4;
parD.x0    = 50;

parD.u1    = 5;
parD.u2    = 6.5;
parD.u3    = 11;

save('parD.mat','parD')

%%
dim.Ts     = 0.20;  % in [h]
dim.t      = 10;
dim.Np     = 4;     % prediction horizon
dim.Nc     = 2;     % control horizon
dim.Wb1    = 3;     % weight in cost function battery 1
dim.Wb2    = 4;     % weight in cost function battery 2
dim.Wd     = 10;    % weight in cost function diesel generator
dim.Wfuel  = 4;     % weight in cost function fuel
dim.We     = 0.4;   % weight in cost function e?

save('dim.mat','dim')

%% Defining battery with matrices
% Defining MLD matrices battery 1
MLDB1.A  = 1;
MLDB1.B1 = -dim.Ts*parB.eta_d(1);
MLDB1.B2 = 0;
MLDB1.B3 = dim.Ts*(parB.eta_d(1)-parB.eta_c(1));
MLDB1.B4 = 0;

MLDB1.E1 = [0; 0; 0; 0; -1; 1; 0; 0; 0; 0]; % E1 matrix battery 1
MLDB1.E2 = [1; -1; 1; -1; 0; 0; 0; 0; -1; 1]; % E2 matrix battery 1
MLDB1.E3 = [0; 0; parB.u_up(1); parB.u_low(1)-eps; 0; 0; -parB.u_up(1); parB.u_low(1); -parB.u_low(1); parB.u_up(1)]; % E3 matrix battery 1
MLDB1.E4 = [0; 0; 0; 0; 0; 0; 1; -1; 1; -1]; % E4 matrix battery 1
MLDB1.g5 = [parB.u_up(1); -parB.u_low(1); parB.u_up(1); -eps; 0; parB.x_up(1); 0; 0; -parB.u_low(1); parB.u_up(1)]; % g5 matrix battery 1

save('MLDB1.mat','MLDB1')

% Defining MLD matrices battery 2
MLDB2.A = 1;
MLDB2.B1 = -dim.Ts*parB.eta_c(1);
MLDB2.B2 = 0;
MLDB2.B3 = dim.Ts*(parB.eta_d(1)-parB.eta_c(1));
MLDB2.B4 = 0;

MLDB2.E1 = [0; 0; 0; 0; -1; 1; 0; 0; 0; 0]; % E1 matrix battery 2
MLDB2.E2 = [1; -1; 1; -1; 0; 0; 0; 0; -1; 1]; % E2 matrix battery 2
MLDB2.E3 = [0; 0; parB.u_up(2); parB.u_low(2)-eps; 0; 0; -parB.u_up(2); parB.u_low(2); -parB.u_low(2); parB.u_up(2)]; % E3 matrix battery 2
MLDB2.E4 = [0; 0; 0; 0; 0; 0; 1; -1; 1; -1]; % E4 matrix battery 2
MLDB2.g5 = [parB.u_up(2); -parB.u_low(2); parB.u_up(2); -eps; 0; parB.x_up(2); 0; 0; -parB.u_low(2); parB.u_up(2)]; % g5 matrix battery 2

save('MLDB2.mat','MLDB2')

%% Defining MLD system diesel generator Jordan
MLDD.A = 1;
MLDD.B1 = zeros(1,1);
MLDD.B2 = dim.Ts*[-parD.a1 -parD.a2 -parD.a3 -parD.a4];
MLDD.B3 = dim.Ts*[-parD.b1 -parD.b2 -parD.b3 -parD.b4];
MLDD.B4 = dim.Ts*parD.Rf;

% Constraint matrices diesel generator
MLDD.E1 = [0 0 0 -1 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0]';
MLDD.E2 = [1 -1 0  0 0 0 0 -1 1 -1 1 0 0 -1 1 -1 1 0 0 -1 1 -1 1 0 0 -1 1 -1 1]';
MLDD.E3 = [0 0 1 0 0 -parD.u_up parD.u_low -parD.u_low parD.u_up eps -(parD.u1-eps)+parD.u_up 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 -parD.u_up parD.u_low -parD.u_low parD.u_up parD.u1 -(parD.u2-eps)+parD.u_up 0 0 0 0 0 0 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -parD.u_up parD.u_low -parD.u2 parD.u3 parD.u2 -(parD.u3-eps)+parD.u_up 0 0 0 0 0 0;
           0 0 1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 -parD.u_up parD.u_low -parD.u_low parD.u_up parD.u3 -parD.u_up]';
MLDD.E4 = [0 0 0 0 0 1 -1 1 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 0 0 0 1 -1 1 -1 0 0 0 0 0 0 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 1 -1 0 0 0 0 0 0 0 0;
           0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 1 -1 1 -1 0 0]';
MLDD.g5 = [parD.u_up; eps; 1; -parD.x_low; parD.x_up; 0; 0; -parD.u_low; parD.u_up; 0; parD.u_up; 0; 0; -parD.u_low; parD.u_up; 0; parD.u_up; 0; 0; -parD.u_low; parD.u_up; 0; parD.u_up; 0; 0; -parD.u_low; parD.u_up; 0; 0];

save('MLDD.mat','MLDD')

