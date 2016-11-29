function [u,up,ui,ud,error,int] = calculate_gains(k,time_params,pid_gains,u,up,ui,ud,error,int,behavior)

    % Behavior Code
    % 1: control robot heading to path heading within path heading tolerance
    % 2: drive straight towards waypoint, control velocity
    % 3: control robot surge and sway velocity to zero within ST
    % 4: control robot heading to goal heading within GHT
    % 5: control robot to location
    
    limit_mode = 1;
    if behavior(1,k) == 1
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k);                                  % correct path heading to within achieved heading
        up(2,k) = -pid_gains(4)*error(5,k);
        up(3,k) = -pid_gains(4)*error(5,k);
        up(4,k) = pid_gains(4)*error(5,k);
        % Integral gain
        if k > 1
            int(1,k) = int(1,k-1)+error(1,k)*time_params(1);                % increment x position goal error integrator
            int(2,k) = int(2,k-1)+error(2,k)*time_params(1);                % increment y position goal error integrator
            int(3,k) = int(3,k-1)+error(3,k)*time_params(1);                % increment psi heading goal error integrator
            int(4,k) = int(4,k-1)+error(4,k)*time_params(1);                % increment path distance error integrator
            int(5,k) = int(5,k-1)+error(5,k)*time_params(1);                % increment path heading error integrator
            int(6,k) = int(6,k-1)+error(6,k)*time_params(1);                % increment surge velocity max speed error integrator
            int(7,k) = int(7,k-1)+error(7,k)*time_params(1);                % increment sway velocity max speed error integrator
            int(8,k) = int(8,k-1)+error(8,k)*time_params(1);                % increment surge velocity stop error integrator
            int(9,k) = int(9,k-1)+error(9,k)*time_params(1);                % increment sway velocity stop error integrator
            ui(1,k) = pid_gains(5)*(int(5,k));
            ui(2,k) = -pid_gains(5)*(int(5,k));
            ui(3,k) = -pid_gains(5)*(int(5,k));
            ui(4,k) = pid_gains(5)*(int(5,k));
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1));
        end
        limit_mode = 2;
    elseif behavior(1,k) == 2
        % Proportional gain
        up(1,k) = pid_gains(7)*error(6,k);
        up(2,k) = pid_gains(7)*error(6,k);
        up(3,k) = pid_gains(7)*error(6,k);
        up(4,k) = pid_gains(7)*error(6,k);
        % Integral gain
        if k > 1
            int(1,k) = int(1,k-1)+error(1,k)*time_params(1);                % increment x position goal error integrator
            int(2,k) = int(2,k-1)+error(2,k)*time_params(1);                % increment y position goal error integrator
            int(3,k) = int(3,k-1)+error(3,k)*time_params(1);                % increment psi heading goal error integrator
            int(4,k) = int(4,k-1)+error(4,k)*time_params(1);                % increment path distance error integrator
            int(5,k) = int(5,k-1)+error(5,k)*time_params(1);                % increment path heading error integrator
            int(6,k) = int(6,k-1)+error(6,k)*time_params(1);                % increment surge velocity max speed error integrator
            int(7,k) = int(7,k-1)+error(7,k)*time_params(1);                % increment sway velocity max speed error integrator
            int(8,k) = int(8,k-1)+error(8,k)*time_params(1);                % increment surge velocity stop error integrator
            int(9,k) = int(9,k-1)+error(9,k)*time_params(1);                % increment sway velocity stop error integrator
            ui(1,k) = pid_gains(8)*(int(6,k));
            ui(2,k) = pid_gains(8)*(int(6,k));
            ui(3,k) = pid_gains(8)*(int(6,k));
            ui(4,k) = pid_gains(8)*(int(6,k));
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(2,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(3,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(4,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 3
        % Proportional gain
        up(1,k) = -pid_gains(7)*error(8,k)+pid_gains(4)*error(9,k);
        up(2,k) = -pid_gains(7)*error(8,k)-pid_gains(4)*error(9,k);
        up(3,k) = -pid_gains(7)*error(8,k)+pid_gains(4)*error(9,k);
        up(4,k) =pid_gains(7)*error(8,k)-pid_gains(4)*error(9,k);
        % Integral gain
        if k > 1
            int(1,k) = int(1,k-1)+error(1,k)*time_params(1);                % increment x position goal error integrator
            int(2,k) = int(2,k-1)+error(2,k)*time_params(1);                % increment y position goal error integrator
            int(3,k) = int(3,k-1)+error(3,k)*time_params(1);                % increment psi heading goal error integrator
            int(4,k) = int(4,k-1)+error(4,k)*time_params(1);                % increment path distance error integrator
            int(5,k) = int(5,k-1)+error(5,k)*time_params(1);                % increment path heading error integrator
            int(6,k) = int(6,k-1)+error(6,k)*time_params(1);                % increment surge velocity max speed error integrator
            int(7,k) = int(7,k-1)+error(7,k)*time_params(1);                % increment sway velocity max speed error integrator
            int(8,k) = int(8,k-1)+error(8,k)*time_params(1);                % increment surge velocity stop error integrator
            int(9,k) = int(9,k-1)+error(9,k)*time_params(1);                % increment sway velocity stop error integrator
            ui(1,k) = -pid_gains(8)*(int(8,k))+pid_gains(5)*(int(9,k));
            ui(2,k) = -pid_gains(8)*(int(8,k))-pid_gains(5)*(int(9,k));
            ui(3,k) = -pid_gains(8)*(int(8,k))+pid_gains(5)*(int(9,k));
            ui(4,k) = -pid_gains(8)*(int(8,k))-pid_gains(5)*(int(9,k));
        end
        % Derivative gain
        if k > 1
            ud(1,k) = -pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1))+...
                pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(2,k) = -pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1))-...
                pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(3,k) = -pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1))+...
                pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(4,k) = -pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1))-...
                pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 4
        % Proportional gain
        up(1,k) = pid_gains(4)*error(3,k);                                  % correct path heading to within achieved heading
        up(2,k) = -pid_gains(4)*error(3,k);
        up(3,k) = -pid_gains(4)*error(3,k);
        up(4,k) = pid_gains(4)*error(3,k);
        % Integral gain
        if k > 1
            int(1,k) = int(1,k-1)+error(1,k)*time_params(1);                % increment x position goal error integrator
            int(2,k) = int(2,k-1)+error(2,k)*time_params(1);                % increment y position goal error integrator
            int(3,k) = int(3,k-1)+error(3,k)*time_params(1);                % increment psi heading goal error integrator
            int(4,k) = int(4,k-1)+error(4,k)*time_params(1);                % increment path distance error integrator
            int(5,k) = int(5,k-1)+error(5,k)*time_params(1);                % increment path heading error integrator
            int(6,k) = int(6,k-1)+error(6,k)*time_params(1);                % increment surge velocity max speed error integrator
            int(7,k) = int(7,k-1)+error(7,k)*time_params(1);                % increment sway velocity max speed error integrator
            int(8,k) = int(8,k-1)+error(8,k)*time_params(1);                % increment surge velocity stop error integrator
            int(9,k) = int(9,k-1)+error(9,k)*time_params(1);                % increment sway velocity stop error integrator
            ui(1,k) = pid_gains(5)*(int(3,k));
            ui(2,k) = -pid_gains(5)*(int(3,k));
            ui(3,k) = -pid_gains(5)*(int(3,k));
            ui(4,k) = pid_gains(5)*(int(3,k));
        end
        % Derivative gain
        if k > 1
%         up(1,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));                                  % correct path heading to within achieved heading
%         up(2,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
%         up(3,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));
%         up(4,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
            ud(1,k) = pid_gains(6)*((error(3,k)-error(3,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(3,k)-error(3,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(3,k)-error(3,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(3,k)-error(3,k-1))/time_params(1));
        end
        limit_mode = 2;
    else
        % error(4,k) distance error
        % error(5,k) heading error
        
%         error(4,k) = sqrt((goal_vars(1)-x(1,k))^2+...       % path distance error
%         (goal_vars(2)-x(2,k))^2);   
%         error(5,k) = atan2(goal_vars(2)-x(2,k),...          % path heading error
%         goal_vars(1)-x(1,k))-x(3,k);
        
%         atan2(goal_vars(2)-x(2,k),goal_vars(1)-x(1,k))

        % Proportional gain
        up(1,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));                                  % correct path heading to within achieved heading
        up(2,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        up(3,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));
        up(4,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        % Integral gain
        if k > 1
            int(1,k) = int(1,k-1)+error(1,k)*time_params(1);                % increment x position goal error integrator
            int(2,k) = int(2,k-1)+error(2,k)*time_params(1);                % increment y position goal error integrator
            int(3,k) = int(3,k-1)+error(3,k)*time_params(1);                % increment psi heading goal error integrator
            int(4,k) = int(4,k-1)+error(4,k)*time_params(1);                % increment path distance error integrator
            int(5,k) = int(5,k-1)+error(5,k)*time_params(1);                % increment path heading error integrator
            int(6,k) = int(6,k-1)+error(6,k)*time_params(1);                % increment surge velocity max speed error integrator
            int(7,k) = int(7,k-1)+error(7,k)*time_params(1);                % increment sway velocity max speed error integrator
            int(8,k) = int(8,k-1)+error(8,k)*time_params(1);                % increment surge velocity stop error integrator
            int(9,k) = int(9,k-1)+error(9,k)*time_params(1);                % increment sway velocity stop error integrator
            ui(1,k) = pid_gains(2)*int(4,k)*(-sin(int(5,k))+cos(int(5,k)));                                  % correct path heading to within achieved heading
            ui(2,k) = pid_gains(2)*int(4,k)*(sin(int(5,k))+cos(int(5,k)));
            ui(3,k) = pid_gains(2)*int(4,k)*(-sin(int(5,k))+cos(int(5,k)));
            ui(4,k) = pid_gains(2)*int(4,k)*(sin(int(5,k))+cos(int(5,k)));
%             ui(1,k) = pid_gains(2)*(int(3,k));
%             ui(2,k) = -pid_gains(2)*(int(3,k));
%             ui(3,k) = -pid_gains(2)*(int(3,k));
%             ui(4,k) = pid_gains(2)*(int(3,k));
        end
        % Derivative gain
        if k > 1
        up(1,k) = pid_gains(6)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));                                  % correct path heading to within achieved heading
        up(2,k) = pid_gains(6)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        up(3,k) = pid_gains(6)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));
        up(4,k) = pid_gains(6)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        end
        limit_mode = 2;
    end
    
    % Total gain vector
    u(1,k) = up(1,k)+ui(1,k)+ud(1,k);
    u(2,k) = up(2,k)+ui(2,k)+ud(2,k);
    u(3,k) = up(3,k)+ui(3,k)+ud(3,k);
    u(4,k) = up(4,k)+ui(4,k)+ud(4,k);
    
    % Limit thrust control effort
    u = limit_thrust(u,k,limit_mode);
        
end