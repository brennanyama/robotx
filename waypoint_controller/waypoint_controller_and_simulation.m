% Created by: Brennan Yamamoto
% Last update: 2016.11.25
% Do not distribute.  Do not duplicate. 

function waypoint_controller_and_simulation_discrete_calculus()

    % Clear workspace
    close all;
    clc;
    
    % Simulation setup
    [time_params,lumped_params,geometry_params,pid_gains,control_tolerances,x,y,snr,rr,meas,u,up,ui,ud,error,int,behavior] = sim_setup();
    
    % Set goal variables
    goal_vars = goal_vars();
    
    % Controller
    for k = 1:1:time_params(3)
        
        % Calculate error matrix
        error = update_error(k,x,meas,goal_vars,error);
        
        % Determine robot behavior based on robot state
        behavior = select_behavior(k,x,control_tolerances,error,behavior);
        
        % Calculate error and PID gains for behavior
        [u,up,ui,ud,error,int] = calculate_gains(k,time_params,pid_gains,u,up,ui,ud,error,int,behavior);
        
        % Simulate plant
        [x,y] = plant(k,time_params,x,y,u,lumped_params,geometry_params);
        
        % Simulate sensor inputs
        meas = sensors(y,meas,k,time_params,snr,rr);
        
%         % Unscented kalman filter
%         [x_est,y_est] = observer(k,time_params,y,x_est,y_est,u,lumped_params,geometry_params,L);

    end
    
    % Plot results
    plot_results(time_params,goal_vars,control_tolerances,x,y,meas,u,up,ui,ud,error,behavior);
    
    % Assign relevant stuff to workspace
    assignin('base','x',x)
    assignin('base','y',y)
    assignin('base','meas',meas)
    assignin('base','error',error)
    assignin('base','u',u)
    assignin('base','behavior',behavior)
    
end

function [time_params,lumped_params,geometry_params,pid_gains,control_tolerances,x,y,snr,rr,meas,u,up,ui,ud,error,int,behavior] = sim_setup()

    % Time variables
    dt = .01;                       % simulation time step [s]
    tend = 200;                     % simulation end time [s]
    N = round(tend/dt);             % number of steps [ ]
    time_params = [dt,tend,N];      % output matrix
    
    % Model lumped parameters
    m = 225;                            % mass [kg]
    I = 100;                            % inertia [kg*m^2]
    bu = 25;                            % surge (longitudinal) drag [N*s/m]
    bv = 400;                           % sway (transverse) drag [N*s/m]
    bpsi = 400;                         % yaw (heading) drag [N*s/(m*rad)]
    lumped_params = [m,I,bu,bv,bpsi];   % output matrix
    
    % Model geometry
    theta = deg2rad(30);            % thruster mount angle [rad]
    w = 1.5;                        % width between thrusters [m]
    l = 3;                          % length between thrusters [m]
    geometry_params = [theta,w,l];  % output matrix
    
    % Controller gains
    kp_pos = 100;
    ki_pos = 1;
    kd_pos = 5;
    kp_psi = 150;    
    ki_psi = 1;     
    kd_psi = 3;
    kp_vel = 200;
    ki_vel = 1;
    kd_vel = 3;
    pid_gains = [kp_pos,ki_pos,kd_pos,...
        kp_psi,ki_psi,kd_psi,...
        kp_vel,ki_vel,kd_vel];
    
    % Control tolerances
    AR = 5;                                             % approach radius [m]
    GR = 0.5;                                           % goal radius [m]
    PHT = deg2rad(10);                                  % path heading tolerance [rad]
    GHT = deg2rad(5);                                   % goal heading tolerance [rad]
    VT = 0.1;                                           % velocity tolerance [m/s]
    control_tolerances = [AR,GR,PHT,GHT,VT];            % output matrix
        
    % Initialize state and output matrix
    x = zeros(6,N);
    x(1,1) = 0;         % x-position initial condition
    x(2,1) = 0;         % y-position initial condition
    x(3,1) = 0;         % psi (heading) initial condition
    x(4,1) = 0;         % surge (forward) velocity initial condition
    x(5,1) = 0;         % sway (transverse) velocity initial condition
    x(6,1) = 0;         % psi (heading) velocity initial condition
    y = zeros(9,N);
    
    % Sensor variables
    snr_pos = 0.5;                      % signal to noise ratio [ ]
    snr_vel = 0;
    snr_acc = 40;
    snr = [snr_pos,snr_vel,snr_acc];
    rr_pos = 40;                        % refresh rate [Hz]
    rr_vel = 0;
    rr_acc = 50;
    rr = [rr_pos,rr_vel,rr_acc];
    
    % Create tracking matrices
    meas = zeros(9,N);      % measurement matrix
    up = zeros(4,N);        % thurster proportional gain [N]
    ui = zeros(4,N);        % thurster integral gain [N]
    ud = zeros(4,N);        % thurster derivative gain [N]
    u = zeros(4,N);         % thurster total gain [N]
    error = zeros(12,N);    % control variable error
    int = zeros(12,N);      % control variable integrator
    behavior = zeros(1,N);  % behavior tracking matrix

end

function goal_vars = goal_vars()
   
    x_goal = 10;                            % goal position [m]  
    y_goal = 10;                            % goal position [m]
    psi_goal = deg2rad(180);                % goal heading [rad]
    goal_vars = [x_goal,y_goal,psi_goal];   % output matrix

end

function error = update_error(k,x,meas,goal_vars,error)

    % Update errors
    error(1,k) = goal_vars(1)-x(1,k);                   % x position goal error
    error(2,k) = goal_vars(2)-x(2,k);                   % y position goal error
    error(3,k) = goal_vars(3)-x(3,k);                   % psi heading goal error
    error(4,k) = sqrt((goal_vars(1)-x(1,k))^2+...       % path distance error
        (goal_vars(2)-x(2,k))^2);   
    error(5,k) = atan2(sin(atan2(goal_vars(2)-x(2,k),...    % HO FUCK EQUATION (there's a good story behind this one)
        goal_vars(1)-x(1,k))-x(3,k)),...
        cos(atan2(goal_vars(2)-x(2,k),...
        goal_vars(1)-x(1,k))-x(3,k)));
    error(6,k) = 0-x(4,k);                              % surge velocity stop error   
    error(7,k) = 0.10-x(4,k);                           % surge velocity slow error
    error(8,k) = 0.25-x(4,k);                           % surge velocity medium slow m/s error
    error(9,k) = 1.0-x(4,k);                            % surge velocity medium fast m/s error
    error(10,k) = 2.00-x(4,k);                          % surge velocity fast m/s error
    error(11,k) = 1.0 - x(5,k);                         % sway velocity fast m/s error
    error(12,k) = atan2(sin(atan2(goal_vars(2)-x(2,k),...    % HO FUCK EQUATION (there's a good story behind this one)
        goal_vars(1)-x(1,k))-x(3,k)),...
        cos(atan2(goal_vars(2)-x(2,k),...
        goal_vars(1)-x(1,k))-x(3,k)))+deg2rad(-90);
    
end

function behavior = select_behavior(k,x,control_tolerances,error,behavior)

%% Waypoint Navigation Code

%     if (error(4,k) >= control_tolerances(1))                % robot is outside the approach radius
%         if (error(5,k) >= control_tolerances(3))            % robot is outside path heading tolerance
%             behavior(1,k) = 1;                                  % control psi to path heading and control u to medium fast
%         else                                                % robot is inside path heading tolerance
%             behavior(1,k) = 2;                                  % control psi to path heading and control u to fast
%         end
%     else                                                    % robot is inside approach radius
%         if (error(4,k) >= control_tolerances(2))            % robot is outside in the goal radius
%             if (error(5,k) >= control_tolerances(3))        % robot is outside path heading tolerance
%                 behavior(1,k) = 3;                              % control psi to path heading and control u to slow
%             else                                            % robot is inside path heading tolerance
%                 behavior(1,k) = 4;                              % control psi to path heading and control u to medium slow
%             end
%         else                                                % robot is inside goal radius
%             if (x(4,k) >= 0.25)                             % robot is moving
%                 behavior(1,k) = 5;                              % control u to 0 m/s
%             else                                            % robot is largely stopped
%                 if (error(3,k) >= control_tolerances(4))    % robot is not facing goal heading
%                     behavior(1,k) = 6;                          % control psi to goal heading and control u to 0 m/s
%                 else                                        % robot is facing goal heading
%                     behavior(1,k) = 7;                          % station keep
%                 end
%             end
%         end
%     end
    
%% Station Keeping (Hold Position) Code

%     behavior(1,k) = 6;

%% Control psi to Goal Heading and Control u to 1 m/s 

%     behavior(1,k) = 8;

%% Control psi to Goal heading and drive straight at set duty cycle

%     behavior(1,k) = 9;

%% Strafe at 90 degrees

    behavior(1,k) = 10;

end

function [u,up,ui,ud,error,int] = calculate_gains(k,time_params,pid_gains,u,up,ui,ud,error,int,behavior)

    % Behavior Code:
    % 1: control psi to path heading and control u to 1 m/s
    % 2: control psi to path heading and control u to 3 m/s
    % 3: control psi to path heading and control u to 0.25 m/s
    % 4: control psi to path heading and control u to 0.75 m/s
    % 5: control u to 0 m/s
    % 6: control psi to goal heading and control u to 0 m/s
    % 7: control x and y position to goal position (station keep)
    % 8: control psi to goal heading and control u to 1 m/s
    % 9: control psi to goal heading and drive straight at set duty cycle
    % 10: control psi to -90 deg from path heading, and control v to 1 m/s
    
    if behavior(1,k) == 1   % control psi to path heading and control u to 1 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 2   % control psi to path heading and control u to 3 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(10,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(10,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(10,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(10,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(10,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(10,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(10,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(10,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(10,k)-error(10,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(10,k)-error(10,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(10,k)-error(10,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(10,k)-error(10,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 3   % control psi to path heading and control u to 0.25 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(7,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(7,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(7,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(7,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(7,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(7,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(7,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(7,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(7,k)-error(7,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(7,k)-error(7,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(7,k)-error(7,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(7,k)-error(7,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 4   % control psi to path heading and control u to 0.75 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(8,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(8,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(8,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(8,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(8,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(8,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(8,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(8,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(8,k)-error(8,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 5   % control u to 0 m/s
        % Proportional gain
        up(1,k) = pid_gains(7)*error(6,k);
        up(2,k) = pid_gains(7)*error(6,k);
        up(3,k) = pid_gains(7)*error(6,k);
        up(4,k) = pid_gains(7)*error(6,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(8)*int(6,k);
            ui(2,k) = pid_gains(8)*int(6,k);
            ui(3,k) = pid_gains(8)*int(6,k);
            ui(4,k) = pid_gains(8)*int(6,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(2,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(3,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(4,k) = pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
        end
        limit_mode = 2;
    elseif behavior(1,k) == 6   % control psi to goal heading and control u to 0 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(6,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(6,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(6,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(6,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(6,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(6,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(6,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(6,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(6,k)-error(6,k-1))/time_params(1));
        end
        limit_mode = 2;
    elseif behavior(1,k) == 7   % control psi to goal heading and control u to 0 m/s
        % Proportional gain
        up(1,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));                                  % correct path heading to within achieved heading
        up(2,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        up(3,k) = pid_gains(1)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));
        up(4,k) = pid_gains(1)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(2)*int(4,k)*(-sin(int(5,k))+cos(int(5,k)));                                  % correct path heading to within achieved heading
            ui(2,k) = pid_gains(2)*int(4,k)*(sin(int(5,k))+cos(int(5,k)));
            ui(3,k) = pid_gains(2)*int(4,k)*(-sin(int(5,k))+cos(int(5,k)));
            ui(4,k) = pid_gains(2)*int(4,k)*(sin(int(5,k))+cos(int(5,k)));
        end
        % Derivative gain
        if k > 1
        up(1,k) = pid_gains(6)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));                                  % correct path heading to within achieved heading
        up(2,k) = pid_gains(6)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        up(3,k) = pid_gains(6)*error(4,k)*(-sin(error(5,k))+cos(error(5,k)));
        up(4,k) = pid_gains(6)*error(4,k)*(sin(error(5,k))+cos(error(5,k)));
        end
        limit_mode = 2;
    elseif behavior(1,k) == 8       % control psi to goal heading and control u to 1 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(2,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(3,k) = -pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        up(4,k) = pid_gains(4)*error(5,k)+pid_gains(7)*error(9,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(2,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(3,k) = -pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
            ui(4,k) = pid_gains(5)*int(5,k)+pid_gains(8)*int(9,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
        end
        limit_mode = 1;
    elseif behavior(1,k) == 9        % control psi to goal heading and drive straight at set duty cycle
            % Proportional gain
        up(1,k) = pid_gains(4)*error(5,k)+25;
        up(2,k) = -pid_gains(4)*error(5,k)+25;
        up(3,k) = -pid_gains(4)*error(5,k)+25;
        up(4,k) = pid_gains(4)*error(5,k)+25;
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(5,k);
            ui(2,k) = -pid_gains(5)*int(5,k);
            ui(3,k) = -pid_gains(5)*int(5,k);
            ui(4,k) = pid_gains(5)*int(5,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(5,k)-error(5,k-1))/time_params(1))+pid_gains(9)*((error(9,k)-error(9,k-1))/time_params(1));
        end
        limit_mode = 1;
    else        % 10: control psi to path heading and control u to 1 m/s
        % Proportional gain
        up(1,k) = pid_gains(4)*error(12,k)-pid_gains(7)*error(11,k);
        up(2,k) = -pid_gains(4)*error(12,k)+pid_gains(7)*error(11,k);
        up(3,k) = -pid_gains(4)*error(12,k)-pid_gains(7)*error(11,k);
        up(4,k) = pid_gains(4)*error(12,k)+pid_gains(7)*error(11,k);
        % Integral gain
        if k > 1
            int = increment_integrator(int,k,error,time_params);
            ui(1,k) = pid_gains(5)*int(12,k)-pid_gains(8)*int(11,k);
            ui(2,k) = -pid_gains(5)*int(12,k)+pid_gains(8)*int(11,k);
            ui(3,k) = -pid_gains(5)*int(12,k)-pid_gains(8)*int(11,k);
            ui(4,k) = pid_gains(5)*int(12,k)+pid_gains(8)*int(11,k);
        end
        % Derivative gain
        if k > 1
            ud(1,k) = pid_gains(6)*((error(12,k)-error(12,k-1))/time_params(1))-pid_gains(9)*((error(11,k)-error(11,k-1))/time_params(1));
            ud(2,k) = -pid_gains(6)*((error(12,k)-error(12,k-1))/time_params(1))+pid_gains(9)*((error(11,k)-error(11,k-1))/time_params(1));
            ud(3,k) = -pid_gains(6)*((error(12,k)-error(12,k-1))/time_params(1))-pid_gains(9)*((error(11,k)-error(11,k-1))/time_params(1));
            ud(4,k) = pid_gains(6)*((error(12,k)-error(12,k-1))/time_params(1))+pid_gains(9)*((error(11,k)-error(11,k-1))/time_params(1));
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

function u = limit_thrust(u,k,limit_mode)

    % mode 1: limit front thruster to 55lb, rear thrusters to 60lb.
    % mode 2: limit all thrusters to 55 lb.
    
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

function int = increment_integrator(int,k,error,time_params)

    int(1,k) = int(1,k-1)+error(1,k)*time_params(1);
    int(2,k) = int(2,k-1)+error(2,k)*time_params(1);
    int(3,k) = int(3,k-1)+error(3,k)*time_params(1);
    int(4,k) = int(4,k-1)+error(4,k)*time_params(1);
    int(5,k) = int(5,k-1)+error(5,k)*time_params(1);
    int(6,k) = int(6,k-1)+error(6,k)*time_params(1);
    int(7,k) = int(7,k-1)+error(7,k)*time_params(1);
    int(8,k) = int(8,k-1)+error(8,k)*time_params(1);
    int(9,k) = int(9,k-1)+error(9,k)*time_params(1);
    int(10,k) = int(10,k-1)+error(10,k)*time_params(1);
    int(11,k) = int(11,k-1)+error(11,k)*time_params(1);
    int(12,k) = int(12,k-1)+error(12,k)*time_params(1);

end

function N = lb2N(lb)

    N = lb*4.4482;    

end

function [x,y] = plant(k,time_params,x,y,u,lumped_params,geometry_params)

    % Define lumped and geometry parameters
    m = lumped_params(1);
    I = lumped_params(2);
    bu = lumped_params(3);
    bv = lumped_params(4);
    bpsi = lumped_params(5);
    theta = geometry_params(1);
    w = geometry_params(2);
    l = geometry_params(3);
    
    % Plant
    A = [0 0 0 cos(x(3,k)) -sin(x(3,k)) 0;...                               % continuous state (system) matrix
            0 0 0 sin(x(3,k)) cos(x(3,k)) 0;...
            0 0 0 0 0 1;...
            0 0 0 -bu/m 0 0;...
            0 0 0 0 -bv/m 0;...
            0 0 0 0 0 -bpsi/I];
    B = [0 0 0 0;...                                                        % continuous control input matrix
        0 0 0 0;...
        0 0 0 0;...
        cos(theta)/m cos(theta)/m cos(theta)/m cos(theta)/m;...
        -sin(theta)/m sin(theta)/m -sin(theta)/m sin(theta)/m;...
        (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I)...
        -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I)...
        -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I)...
        (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I)];
    C = [eye(6);A(4:6,:)];                                                  % continuous output matrix
    [~,Bc] = size(B);
    [Cr,~] = size(C);
    D = zeros(Cr,Bc);                                                       % continuous feedthrough (direct transmission) matrix
    F = eye(6)+A*time_params(1);                                            % discrete state (system) matrix
    G = B*time_params(1);                                                   % discrete control input matrix
    H = C;                                                                  % discrete output matrix
    J = D;                                                                  % discrete feedthrough (direct transmission) matrix
    if k ~= time_params(3)
        x(:,k+1) = F*x(:,k)+G*[u(1,k);u(2,k);u(3,k);u(4,k)];                % state matrix integration solution
        y(:,k+1) = H*x(:,k)+J*[u(1,k);u(2,k);u(3,k);u(4,k)];                % output matrix integration solution
    end
    
end

function meas = sensors(y,meas,k,time_params,snr,rr)

    dt = time_params(1);                            % time step

    if k ~= time_params(3)
        % Position
        if mod(k*dt,1/rr(1)) == 0                           % update sensor reading at rr
            meas(1:3,k+1) = awgn(y(1:3,k+1),snr(1));        % add noise
        else
            meas(1:3,k+1) = meas(1:3,k);
        end

        % Velocity
        if mod(k*dt,1/rr(2)) == 0                           % update sensor reading at rr
            meas(4:6,k+1) = awgn(y(4:6,k+1),snr(2));        % add noise
        else
            meas(4:6,k+1) = meas(4:6,k);
        end

        % Acceleration
        if mod(k*dt,1/rr(3)) == 0                           % update sensor reading at rr
            meas(7:9,k+1) = awgn(y(7:9,k+1),snr(3));        % add noise
        else
            meas(7:9,k+1) = meas(7:9,k);
        end
    end
    
end

function [x_est,y_est] = observer(k,time_params,y,x_est,y_est,u,lumped_params,geometry_params,L)

    % Define lumped and geometry parameters
    m = lumped_params(1);
    I = lumped_params(2);
    bu = lumped_params(3);
    bv = lumped_params(4);
    bpsi = lumped_params(5);
    theta = geometry_params(1);
    w = geometry_params(2);
    l = geometry_params(3);

    % Add noise to plant output to emulate sensor measurements
    if k~= time_params(3)
        pos_meas = awgn(y(1:3,k+1),10);             % position data (measured)   
        vel_meas = zeros(3,1);                      % velocity data (not measured)
        acc_meas = awgn(y(7:9,k+1),10);             % acceleration data (measured)
    else
        pos_meas = zeros(3,1);
        vel_meas = zeros(3,1);
        acc_meas = zeros(3,1);
    end
    
    n = 6;              % number of state
    q = 0.1;            % std of process 
    r = 0.1;            % std of measurement
    Q = q^2*eye(n);     % covariance of process
    R = r^2;            % covariance of measurement  
    
    m = 225;                            % mass [kg]
    I = 100;                            % inertia [kg*m^2]
    bu = 25;                            % surge (longitudinal) drag [N*s/m]
    bv = 400;                           % sway (transverse) drag [N*s/m]
    bpsi = 400;                         % yaw (heading) drag [N*s/(m*rad)]
    
    % State matrix
    f = @(x)[cos(x(3))*x(4)-sin(x(3))*x(4);...
        sin(x(3))*x(4)+cos(x(3))*x(4);...
        x(6);...
        -(bu/m)*x(4);...
        -(bv/m)*x(5);...
        -(bpsi/m)*x(6)];                        % nonlinear state equations
    h = @(x)x(1);                               % measurement equation
    s = [0;0;0;0;0;1];                          % initial state
    x = s + q*randn(n,1);                       % initial state with noise
    P = eye(n);                                 % initial state covraiance
    N = 20;                                     % total dynamic steps
    
    % Preallocate memory
    xV = zeros(n,N);                            % estimate
    sV = zeros(n,N);                            % actual
    zV = zeros(1,N);
    
    for k=1:N
      z = h(s) + r*randn;                     % measurments
      sV(:,k)= s;                             % save actual state
      zV(k)  = z;                             % save measurment
      [x, P] = ukf(f,x,P,h,z,Q,R);            % ekf 
      xV(:,k) = x;                            % save estimate
      s = f(s) + q*randn(n,1);                % update process 
    end
    for k=1:n                                 % plot results
      subplot(n,1,k)
      plot(1:N, sV(k,:), '-', 1:N, xV(k,:), '--')
    end
        
%     % State observer
%     A = [0 0 0 cos(pos_meas(3)) -sin(pos_meas(3)) 0;...                     % continuous state (system) matrix
%             0 0 0 sin(pos_meas(3)) cos(pos_meas(3)) 0;...
%             0 0 0 0 0 1;...
%             0 0 0 -bu/m 0 0;...
%             0 0 0 0 -bv/m 0;...
%             0 0 0 0 0 -bpsi/I]; 
%     B = [0 0 0 0;...                                                        % continuous control input matrix
%         0 0 0 0;...
%         0 0 0 0;...
%         cos(theta)/m cos(theta)/m cos(theta)/m cos(theta)/m;...
%         -sin(theta)/m sin(theta)/m -sin(theta)/m sin(theta)/m;...
%         (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I)...
%         -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I)...
%         -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I)...
%         (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I)];
%     C = [eye(6);A(4:6,:)];                                                  % continuous output matrix
%     [~,Bc] = size(B);
%     [Cr,~] = size(C);
%     D = zeros(Cr,Bc);                                                       % continuous feedthrough (direct transmission) matrix
%     F = eye(6)+A*time_params(1);                                            % discrete state (system) matrix
%     G = B*time_params(1);                                                   % discrete control input matrix
%     H = C;                                                                  % discrete output matrix
%     J = D;                                                                  % discrete feedthrough (direct transmission) matrix    
%     if k ~= time_params(3)
%         x_est(:,k+1) = F*x_est(:,k)+...                                     % state estimate integration solution
%             L*([pos_meas;vel_meas;acc_meas]-y_est(:,k))+...
%             G*[u(1,k);u(2,k);u(3,k);u(4,k)];
%         y_est(:,k+1) = H*x_est(:,k)+...                                     % output estimate matrix integration solution
%             J*[u(1,k);u(2,k);u(3,k);u(4,k)];
%     end

end

function plot_results(time_params,goal_vars,control_tolerances,x,y,meas,u,up,ui,ud,error,behavior)

    % Time matrix for plotting
    tt = 0:time_params(1):(time_params(3)-1)*time_params(1);
        
    figure(); 
    subplot(3,3,1);
    plot(tt,y(1,:),'b-',tt,meas(1,:),'r.')
    xlabel('time [s]');
    ylabel('position [m]');
    title('Robot x-position vs. Time');
    grid on;
    subplot(3,3,4);
    plot(tt,y(2,:),'b-',tt,meas(2,:),'r.')
    xlabel('time [s]');
    ylabel('position [m]');
    title('Robot y-position vs. Time');
    grid on;
    subplot(3,3,7);
    plot(tt,rad2deg(y(3,:)),'b-',tt,rad2deg(meas(3,:)),'r.')
    xlabel('time [s]');
    ylabel('heading [deg]');
    title('Robot heading vs. Time');
    grid on;
    subplot(3,3,2);
    plot(tt,y(4,:),'b-',tt,meas(4,:),'r.')
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    title('Robot surge velocity vs. Time');
    grid on;
    subplot(3,3,5);
    plot(tt,y(5,:),'b-',tt,meas(5,:),'r.')
    xlabel('time [s]');
    ylabel('velocity [m/s]');
    title('Robot sway velocity vs. Time');
    grid on;
    subplot(3,3,8);
    plot(tt,rad2deg(y(6,:)),'b-',tt,rad2deg(meas(6,:)),'r.')
    xlabel('time [s]');
    ylabel('angular velocity [deg/s]');
    title('Robot angular velocity vs. Time');
    grid on;
    subplot(3,3,3);
    plot(tt,y(7,:),'b-',tt,meas(7,:),'r.')
    xlabel('time [s]');
    ylabel('acceleration [m/s^2]');
    title('Robot surge acceleration vs. Time');
    grid on;
    subplot(3,3,6);
    plot(tt,y(8,:),'b-',tt,meas(8,:),'r.')
    xlabel('time [s]');
    ylabel('acceleration [m/s^2]');
    title('Robot sway acceleration vs. Time');
    grid on;
    subplot(3,3,9);
    plot(tt,rad2deg(y(9,:)),'b-',tt,rad2deg(meas(9,:)),'r.')
    xlabel('time [s]');
    ylabel('angular acceleration [deg/s^2]');
    title('Robot angular acceleration vs. Time');
    grid on;
    
    figure(); 
    th = 0:pi/50:2*pi;
    xunit_GR = control_tolerances(2) * cos(th) + goal_vars(1);
    yunit_GR = control_tolerances(2) * sin(th) + goal_vars(2);
    xunit_AR = control_tolerances(1) * cos(th) + goal_vars(1);
    yunit_AR = control_tolerances(1) * sin(th) + goal_vars(2);
    plot(x(1,:),x(2,:),'b.-',goal_vars(1),goal_vars(2),'r*',xunit_GR, yunit_GR,'r',xunit_AR,yunit_AR,'g');
    xlabel('x-position [m]');
    ylabel('y-position [m]');
    title('Robot Position');
    axis equal;
    grid on;
    
    figure(); 
    plot(tt,rad2deg(x(3,:)),'b.-')
    xlabel('time [s]');
    ylabel('heading [deg]');
    title('Robot Heading vs. Time');
    grid on;
    
    figure();
    subplot(3,1,1)
    plot(tt,x(4,:),'b.-')
    xlabel('time [s]');
    ylabel('surge velocity [m/s]');
    title('Robot Surge Velocity vs. Time');
    grid on;
    subplot(3,1,2)
    plot(tt,x(5,:),'b.-')
    xlabel('time [s]');
    ylabel('surge velocity [m/s]');
    title('Robot Sway Velocity vs. Time');
    grid on;
    subplot(3,1,3)
    plot(tt,behavior,'b.')
    xlabel('time [s]');
    ylabel('behavior [ ]');
    title('Behavior vs. Time');
    grid on;
    
    figure();
    yyaxis left;                % (must have matlab 2016 to run)
    plot(tt,error(4,:));
    ylabel('distance error [m]');
    yyaxis right;
    plot(tt,rad2deg(error(3,:)));
    xlabel('time [s]');
    ylabel('path error [deg]');
    title('Error vs. Time');
    grid on;
    
    figure();
    subplot(4,1,1);
    plot(tt,u(1,:),'k',tt,up(1,:),'r',tt,ui(1,:),'g',tt,ud(1,:),'b');
    ylabel('T1 [N]');
    title('PID Gains vs. Time');
    grid on;
    subplot(4,1,2);
    plot(tt,u(2,:),'k',tt,up(2,:),'r',tt,ui(2,:),'g',tt,ud(2,:),'b');
    ylabel('T2 [N]');
    grid on;
    subplot(4,1,3);
    plot(tt,u(3,:),'k',tt,up(3,:),'r',tt,ui(3,:),'g',tt,ud(3,:),'b');
    ylabel('T3 [N]');
    grid on;
    subplot(4,1,4);
    plot(tt,u(4,:),'k',tt,up(4,:),'r',tt,ui(4,:),'g',tt,ud(4,:),'b'); 
    xlabel('time [s]');
    ylabel('T4 [N]');
    legend('total','proportional','integral','derivative','Location','east');
    grid on;

end

function [h1, h2] = plot_dir(vX, vY)
    % Adapted from https://www.mathworks.com/matlabcentral/fileexchange/1676-plot-with-direction

    rMag = 0.5;

    % X coordinates of tails of arrows
    vXQ0 = vX(1:end-1);
    % Y coordinates of tails of arrows
    vYQ0 = vY(1:end-1);

    % X coordinates of heads of arrows
    vXQ1 = vX(2:end);
    % Y coordinates of heads of arrows
    vYQ1 = vY(2:end);

    % vector difference between heads & tails
    vPx = (vXQ1 - vXQ0) * rMag;
    vPy = (vYQ1 - vYQ0) * rMag;

    % make plot 
    h1 = plot (vX, vY, '.-');
    hold on;
    
    % add arrows 
    h2 = quiver (vXQ0,vYQ0, vPx, vPy, 0, 'r');
    grid on;
    hold off;
    axis equal;

end
