%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.9
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

%% Loading data
load dim.mat; load MLDB1.mat; load MLDB2.mat; load MLDD.mat; load parB.mat; load parD.mat;
load M1.mat; load M2.mat; load M3.mat; load F1.mat; load F2.mat; load F3.mat;
load W1.mat; load W2.mat; load W3.mat; load W4.mat; load W5.mat; load S1.mat; load S2.mat;

%% Defining Pload (needed for the cost)
% Place Pload in MPC loop.

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

%% Cost function


%% MPC loop for battery 1

for  k = 1:10
    J(k) = W1.W1b1*modelb1.obj(1:dim.Np) + S1.S1b1*modelb1.obj(dim.Np+1:4*dim.Np) + S2.S2b1*modelb1.obj(4*dim.Np+1:end) + Ce(k)*Pload(k) + W3.W3b1*M3.M3b1;
end

