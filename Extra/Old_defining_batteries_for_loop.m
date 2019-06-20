%% Old defining batteries
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
