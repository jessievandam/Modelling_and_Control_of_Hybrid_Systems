function [ f,f_dak ] = PWAapprox( x )
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
a1 = x(1);
a2 = x(2);
a3 = x(3);
a4 = x(4);
b1 = x(5);
b2 = x(6);
b3 = x(7);
b4 = x(8);
u1 = x(9);
u2 = x(10);
u3 = x(11);

for k = 1:1500
    u(k) = k/100;
    if u(k) < 2
        f(k) = u(k)^2 + 4;
    elseif u(k) < 5
        f(k) = 4*u(k);
    elseif u(k) < 7
        f(k) = -9.44*u(k)^3 + 166.06*u(k)^2 - 948.22*u(k) + 1790.28;
    elseif u(k) < 9
        f(k) = -11.78*u(k) + 132.44;
    else
        f(k) = 4.01*(u(k)-10.47)^2 + 17.79;
    end
    
    if u(k) < u1
        f_dak(k) = a1 + b1*u;
    elseif u(k) < u2
        f_dak(k) = a2 + b2*u;
    elseif u(k) < u3
        f_dak(k) = a3 + b3*u;
    else
        f_dak(k) = a4 + b4*u;
    end
end

end

