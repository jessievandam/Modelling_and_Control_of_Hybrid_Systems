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

x = zeros(20,11);
x0 = zeros(1,11);
for i = 1:21
    x0 = [parD.a1+(i-11) parD.a2+(i-11) parD.a3+(i-11) parD.a4+(i-11) parD.b1+(i-11) parD.b2+(i-11) parD.b3+(i-11) parD.b4+(i-11) 4+((i-11)/10) 6.5+((i-11)/10) 11+((i-11)/10)];
    x(i,:) = fmincon(@PWAapprox,x0,A,b);
    fun_int(i) = PWAapprox(x(i,:));
end

fun_min = find(fun_int == min(fun_int(:)));
x_min = x(fun_min,:);

%% Plot real function together with approximation
% real function
step = 0.01;
ud = 0:step:15;

for i = 1:length(ud);
    if i <= round(2/step)
    funreal(i) = ud(i)^2+4;
    elseif i <= round(5/step)
    funreal(i) = 4*ud(i);
    elseif i <= round(7/step)
    funreal(i) = -9.44*ud(i)^3+166.06*ud(i)^2-948.22*ud(i)+1790.28;
    elseif i <= round(9/step)
    funreal(i) = -11.78*ud(i) + 132.44;
    elseif i > round(9/step)
    funreal(i) = 4.01*(ud(i)-10.47)^2+17.79;
    end
end

% approximate function
for i = 1:length(ud);
    if i <= round(x_min(9)/step)
    funapprox(i) = x_min(1)+x_min(5)*ud(i); 
    elseif i <= round(x_min(10)/step)
    funapprox(i) = x_min(2)+x_min(6)*ud(i);     
    elseif i <= round(x_min(11)/step)
    funapprox(i) = x_min(3)+x_min(7)*ud(i);     
    elseif i > round(x_min(11)/step)
    funapprox(i) = x_min(4)+x_min(8)*ud(i); 
    end
end

figure;
hold on; grid on;
plot(ud,funreal);
plot(ud,funapprox);
xlabel('generated power u_d [kW]');
ylabel('consumed fuel of diesel generator [kg/h]');
legend('real fuel consumption', 'approximated fuel consumption');

rmse = rms(funreal-funapprox);