%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Step 2.4
% Jessie van Dam (4395832) and Miranda van Duijn (4355776)
clear all; close all; clc;

parD.a1    = 136/75;
parD.a2    = -22732801477233947 / 247390116249600;
parD.a3    = 3591809146630509439 / 32061759065948160;
parD.a4    = -12101933550265749 / 562949953421312;
parD.b1    = 436 / 125;
parD.b2    = 1357800653035933 / 61847529062400;
parD.b3    = -1472321153042095969 / 160308795329740800;
parD.b4    = 11422592324890509 / 562949953421312;

A = [0 0 0 0 0 0 0 0  0  0  1
     0 0 0 0 0 0 0 0  1 -1  0
     0 0 0 0 0 0 0 0  0  1 -1
     0 0 0 0 0 0 0 0 -1  0  0  ];
 
b = [15; 0; 0; 0];

%x0 = [parD.a1 parD.a2 parD.a3 parD.a4 parD.b1 parD.b2 parD.b3 parD.b4 4 6.5 11];
x0 = [10 -10 10 -10 10 10 -10 10 4 6 11];
% x = fmincon(@PWAapprox,x0,A,b,'MaxFunctionEvaluations',5000);
x = fmincon(@PWAapprox,x0,A,b);

