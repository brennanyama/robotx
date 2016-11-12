function u = limit_thrust(u,k,limit_mode)

    % mode 1: limit front thruster to 55lb, rear thrusters to 60lb.
    % mode 2: limit al thrusters to 55 lb.
    
    if limit_mode == 1
        % Limit max input
        if u(1,k) >= lb2N(55)
            u(1,k) = lb2N(55);
        elseif u(1,k) <= lb2N(-55)
            u(1,k) = lb2N(-55);
        end
        if u(2,k) >= lb2N(55)
            u(2,k) = lb2N(55);
        elseif u(2,k) <= lb2N(-55)
            u(2,k) = lb2N(-55);
        end
        if u(3,k) >= lb2N(60)
            u(3,k) = lb2N(60);
        elseif u(3,k) <= lb2N(-60)
            u(3,k) = lb2N(-60);
        end
        if u(4,k) >= lb2N(60)
            u(4,k) = lb2N(60);
        elseif u(4,k) <= lb2N(-60)
            u(4,k) = lb2N(-60);
        end
    else 
        % Limit max input
        if u(1,k) >= lb2N(55)
            u(1,k) = lb2N(55);
        elseif u(1,k) <= lb2N(-55)
            u(1,k) = lb2N(-55);
        end
        if u(2,k) >= lb2N(55)
            u(2,k) = lb2N(55);
        elseif u(2,k) <= lb2N(-55)
            u(2,k) = lb2N(-55);
        end
        if u(3,k) >= lb2N(55)
            u(3,k) = lb2N(55);
        elseif u(3,k) <= lb2N(-55)
            u(3,k) = lb2N(-55);
        end
        if u(4,k) >= lb2N(55)
            u(4,k) = lb2N(55);
        elseif u(4,k) <= lb2N(-55)
            u(4,k) = lb2N(-55);
        end
    end

end