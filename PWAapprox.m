function [fun_int] = PWAapprox(x)

% input vector x = [a1; a2; a3; a4; b1; b2; b3; b4; u1; u2; u3]

% computing f and f_dak
for k = 1:15000
    u(k) = k/1000;
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
    
    if u(k) < x(9)
        f_dak(k) = x(1) + x(5)*u(k);
    elseif u(k) < x(10)
        f_dak(k) = x(2) + x(6)*u(k);
    elseif u(k) < x(11)
        f_dak(k) = x(3) + x(7)*u(k);
    else
        f_dak(k) = x(4) + x(8)*u(k);
    end
end

% computing squared area between f and f_dak
fun = (f - f_dak).^2;
fun_int = trapz(u,fun);

end

