%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.4
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;
load('parD.mat');

A = [0 0 0 0 0 0 0 0  0  0  1
     0 0 0 0 0 0 0 0  1 -1  0
     0 0 0 0 0 0 0 0  0  1 -1
     0 0 0 0 0 0 0 0 -1  0  0  ];
 
b = [15; 0; 0; 0];

% x0 = [parD.a1 parD.a2 parD.a3 parD.a4 parD.b1 parD.b2 parD.b3 parD.b4 4 6.5 11];
x0 = [2 -100 100 -200 5 30 -10 20 5 5 20];
% x = fmincon(@PWAapprox,x0,A,b,'MaxFunctionEvaluations',5000);
x = fmincon(@PWAapprox,x0,A,b);

