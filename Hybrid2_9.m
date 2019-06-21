%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.9
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath(genpath('C:\Documenten\TU Delft\MSc Systems and Control\Q4\Modelling and Control of Hybrid Systems\Project\Modelling_and_Control_of_Hybrid_Systems'));
addpath c:\gurobi811\win64\matlab\

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

%% MPC loop for battery 1




