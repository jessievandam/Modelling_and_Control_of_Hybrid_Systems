%% SC4160 MODELLING AND CONTROL OF HYBRID SYSTEMS
% Jessie van Dam () and Miranda van Duijn (4355776)
clear all; close all; clc;
addpath C:\Users\Miranda\hysdel-2.0.6-MINGW32_NT-5.1-i686

% Define parameters
parB.eta_c = [0.9 0.95];
parB.eta_d = [0.8 0.77];
parB.x_up  = [48 64];
parB.u_low = [-3 -4];
parB.u_up  = [2 3];
parB.A     = 1;

parD.x_low = 10;
parD.x_up  = 120;
parD.u_up  = 15;
parD.Rf    = 0.4;
parD.a1    = 136/75;
parD.a2    = -22732801477233947 / 247390116249600;
parD.a3    = 3591809146630509439 / 32061759065948160;
parD.a4    = -12101933550265749 / 562949953421312;
parD.b1    = 436 / 125;
parD.b2    = 1357800653035933 / 61847529062400;
parD.b3    = -1472321153042095969 / 160308795329740800;
parD.b4    = 11422592324890509 / 562949953421312;
dim.Ts     = 0.20; % in [h]

dim.t      = 10;

%% Defining battery with matrices
% Defining MLD matrices battery 1
MLDB1.A  = 1;
MLDB1.B1 = -dim.Ts*parB.eta_c(2);
MLDB1.B2 = 0;
MLDB1.B3 = dim.Ts*(parB.eta_d(2)-parB.eta_c(2));
MLDB1.B4 = 0;

MLDB1.E1 = [0; 0; -1; 1; 0; 0; 0; 0]; % E1 matrix battery 1
MLDB1.E2  = [1; -1; 0; 0; 0; 0; -1; 1]; % E2 matrix battery 1
MLDB1.E3  = [parB.u_up(1); parB.u_low(1)-eps; 0; 0; -parB.u_up(1); parB.u_low(1); parB.u_low(1); parB.u_up(1)]; % E3 matrix battery 1
MLDB1.g5  = [parB.u_up(1); -eps; 0; parB.x_up(1); 0; 0; -parB.u_low(1); parB.u_up(1)]; % g5 matrix battery 1

% Defining MLD matrices battery 2
MLDB2.A = 1;
MLDB2.B1 = -dim.Ts*parB.eta_c(1);
MLDB2.B2 = 0;
MLDB2.B3 = dim.Ts*(parB.eta_d(1)-parB.eta_c(1));
MLDB2.B4 = 0;

MLDB2.E1 = [0; 0; -1; 1; 0; 0; 0; 0]; % E1 matrix battery 2
MLDB2.E2 = [1; -1; 0; 0; 0; 0; -1; 1]; % E2 matrix battery 2
MLDB2.E3 = [parB.u_up(2); parB.u_low(2)-eps; 0; 0; -parB.u_up(2); parB.u_low(2); parB.u_low(2); parB.u_up(2)]; % E3 matrix battery 2
MLDB2.g5 = [parB.u_up(2); -eps; 0; parB.x_up(2); 0; 0; -parB.u_low(2); parB.u_up(2)]; % g5 matrix battery 2

%% Defining diesel generator with matrices
MLDD.A = 1;
MLDD.B1 = zeros(4,1);
MLDD.B2 = [-parD.a1 -parD.a2 -parD.a3 -parD.a4];
MLDD.B3 = [-parD.b1 -parD.b2 -parD.b3 -parD.b4];

%% Defining battery as MLD system using a for-loop
x_b1 = zeros(1,dim.t+1);
x_b2 = zeros(1,dim.t+1);
z    = zeros(1,dim.t);

u_b  = [0.5 0 0.1 3 5 1 1 0 1 1];
s_b  = [1 1 1 1 0 0 0 1 0 1];

for k = 1:dim.t
    
    if u_b(k) <= parB.u_up*(1-s_b(k))
        if u_b(k) >= eps + (parB.u_low-eps)*s_b(k)
            if z(k) <= parB.u_up*s_b(k)
                if z(k) >= parB.u_low*s_b(k)
                    if z(k) <= u_b(k) - parB.u_low*(1-s_b(k))
                        if z(k) <= u_b(k) - parB.u_up*(1-s_b(k))
                                z(k) = s_b(k)*u_b(k);
                                x_b1(k+1) = parB.A*x_b1(k) - parB.eta_c(1)*u_b(k) + (parB.eta_d(1)-parB.eta_c(1))*z(k);
                                x_b2(k+1) = parB.A*x_b2(k) - parB.eta_c(2)*u_b(k) + (parB.eta_d(2)-parB.eta_c(2))*z(k);
                        end
                    end
                end
            end
        end
    end

end
