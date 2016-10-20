% To do: should be modified to allow the goal-vars to change on
% the fly. 

function heading_velocity_controller_and_simulation()

    % Clear workspace
    clear all;
    close all;
    clc;
    
    %ros node initiate
    rosinit
    
    
    
    % Simulation setup
    [dt,tend,N,m,I,bu,bv,bpsi,theta,w,l,K,x,T,u,up,ui,ud,error,int] = sim_setup();
    
    % Set goal variables
    [v_goal,psi_goal] = goal_vars();
              
    % Controller
    for k = 1:1:N
        
        % Calculate error and PID gains
        [error,int,u,up,ui,ud] = calculate_gains(k,dt,K,x,v_goal,psi_goal,u,up,ui,ud,error,int);
        
        % Calculate inputs based off of gains
        T = calculate_inputs(T,u,k);
        
        % Simulate Plant
        x = plant(N,x,dt,k,bu,bv,bpsi,m,I,theta,w,l,T);
                
    end
    
    % Plot results
    plot_results(N,dt,x,u,up,ui,ud,error);
        
    % Assign relevant stuff to workspace
    assignin('base','x',x)
    assignin('base','error',error)
    
end

function [dt,tend,N,m,I,bu,bv,bpsi,theta,w,l,K,x,T,u,up,ui,ud,error,int] = sim_setup()

    % Simulation time variables
    dt = .01;           % simulation time step [s]
    tend = 50;          % simulation end time [s]
    N = round(tend/dt); % number of steps [ ]
    
    % Model initial conditions
    x0(1) = 0;          % x-position [m]
    x0(2) = 0;          % y-position [m]
    x0(3) = pi;          % psi (heading) [rad]
    x0(4) = 0;          % surge (forward) velocity [m/s]
    x0(5) = 0;          % sway (transverse) velocity [m/s]
    x0(6) = 0;          % yaw (heading) velocity [rad/s]
    
    % Model lumped parameters
    m = 400;                 % mass [kg]
    I = 300;                 % inertia [kg*m^2]
    bu = 100;                 % surge (longitudinal) drag [N*s/m]
    bv = 400;                % sway (transverse) drag [N*s/m]
    bpsi = 400;               % yaw (heading) drag [N*s/(m*rad)]
    
    % Model geometry
    theta = deg2rad(30);    % thruster mount angle 
    w = 2;                  % width between thrusters [m]
    l = 5;                  % length between thrusters [m]
    
    % Controller gains
    kp = 100;
    ki = 0.03;
    kd = 0.1;
    K = [kp ki kd];
    
    % Initialize state vector
    x = zeros(6,N);
    
    % Insert initial conditions into state vector
    x(1,1) = x0(1);     % x-position 
    x(2,1) = x0(2);     % y-position
    x(3,1) = x0(3);     % psi (heading)
    x(4,1) = x0(4);     % surge (forward) velocity
    x(5,1) = x0(5);     % sway (transverse) velocity
    x(6,1) = x0(6);     % psi (heading)
    
    % Create tracking matrices
    T = zeros(4,N);     % thruster output [N] (max 245 N)
    up = zeros(4,N);    % thurster proportional gain [N]
    ui = zeros(4,N);    % thurster integral gain [N]
    ud = zeros(4,N);    % thurster derivative gain [N]
    u = zeros(4,N);     % thurster total gain [N]
    error = zeros(2,N); % control variable error
    int = zeros(2,N);   % control variable integrator
    
end

function [v_goal,psi_goal] = goal_vars()
   
    v_goal = 2;                 % goal velocity [m/s]
    psi_goal = deg2rad(45);    % goal heading [rad]

end

function [error,int,u,up,ui,ud] = calculate_gains(k,dt,K,x,v_goal,psi_goal,u,up,ui,ud,error,int)
    
        % Calculate error
        error(1,k) = psi_goal-x(3,k);       % heading error
        error(2,k) = v_goal-x(4,k);         % velocity error
        
        % Proportional gain (increases with errors)
        up(1,k) = K(1)*(error(2,k)-error(1,k));
        up(2,k) = K(1)*(error(2,k)+error(1,k));
        up(3,k) = K(1)*(error(2,k)-error(1,k));
        up(4,k) = K(1)*(error(2,k)+error(1,k));    

        % Integral gain (increases with large change in error)
        ui(1,k) = K(2)*(int(2,k))-K(2)*int(1,k);
        ui(2,k) = K(2)*(int(2,k))+K(2)*int(1,k);
        ui(3,k) = K(2)*(int(2,k))-K(2)*int(1,k);
        ui(4,k) = K(2)*(int(2,k))+K(2)*int(1,k);
        int(1,k+1) = int(1,k)+error(1,k);       % increment heading integrator
        int(2,k+1) = int(2,k)+error(2,k);       % increment velocity integrator
        
        % Derivative gain (increases with large change in state)
        if k == 1
            ud(1,k) = 1; ud(2,k) = 1; ud(3,k) = 1; ud(4,k) = 1;
        else
            ud(1,k) = K(3)*(-(x(4,k)+x(4,k-1))/dt-K(3)*(-x(3,k)+x(3,k-1))/dt);
            ud(2,k) = K(3)*(-(x(4,k)+x(4,k-1))/dt+K(3)*(-x(3,k)+x(3,k-1))/dt);
            ud(3,k) = K(3)*(-(x(4,k)+x(4,k-1))/dt-K(3)*(-x(3,k)+x(3,k-1))/dt);
            ud(4,k) = K(3)*(-(x(4,k)+x(4,k-1))/dt+K(3)*(-x(3,k)+x(3,k-1))/dt);
        end
        
        % Total gain vector
        u(1,k) = up(1,k)+ui(1,k)+ud(1,k);
        u(2,k) = up(2,k)+ui(2,k)+ud(2,k);
        u(3,k) = up(3,k)+ui(3,k)+ud(3,k);
        u(4,k) = up(4,k)+ui(4,k)+ud(4,k);
    
end

function T = calculate_inputs(T,u,k)
    chatterpub1 = rospublisher('/motor_q1', 'std_msgs/UInt16')
    chatterpub2 = rospublisher('/motor_q2', 'std_msgs/UInt16')
    chatterpub3 = rospublisher('/motor_q3', 'std_msgs/UInt16')
    chatterpub4 = rospublisher('/motor_q4', 'std_msgs/UInt16')
    % Iterate inputs
    T(1,k) = T(1,k)+u(1,k);
    T(2,k) = T(2,k)+u(2,k);
    T(3,k) = T(3,k)+u(2,k);
    T(4,k) = T(4,k)+u(4,k);

    % Check max input
    if T(1,k) >= lb2N(60)
        T(1,k) == lb2N(60);
    elseif T(1,k) <= lb2N(-60)
        T(1,k) == lb2N(-60);
    end
    if T(2,k) >= lb2N(60)
        T(2,k) == lb2N(60);
    elseif T(2,k) <= lb2N(-60)
        T(2,k) == lb2N(-60);
    end
    if T(3,k) >= lb2N(60)
        T(3,k) == lb2N(60);
    elseif T(3,k) <= lb2N(-60)
        T(3,k) == lb2N(-60);
    end
    if T(4,k) >= lb2N(60)
        T(4,k) == lb2N(60);
    elseif T(4,k) <= lb2N(-60)
        T(4,k) == lb2N(-60);
    end
    
    chattermsg1 = rosmessage(chatterpub1)
    chattermsg2 = rosmessage(chatterpub2)
    chattermsg3 = rosmessage(chatterpub3)
    chattermsg4 = rosmessage(chatterpub4)
    
    chattermsg1.Data = T(1,k)
    chattermsg2.Data = T(2,k)
    chattermsg3.Data = T(3,k)
    chattermsg4.Data = T(4,k)
    
    send(chatterpub1, chattermsg1);
    send(chatterpub2, chattermsg2);
    send(chatterpub3, chattermsg3);
    send(chatterpub4, chattermsg4);
end

function N = lb2N(lb)

    N = lb*4.4482;    
    
end



function x = plant(N,x,dt,k,bu,bv,bpsi,m,I,theta,w,l,T)

    A = [0 0 0 cos(x(3,k)) -sin(x(3,k)) 0;...
            0 0 0 sin(x(3,k)) cos(x(3,k)) 0;...
            0 0 0 0 0 1;...
            0 0 0 -bu/m 0 0;...
            0 0 0 0 -bv/m 0;...
            0 0 0 0 0 -bpsi/I];
    B = [0 0 0 0;...
        0 0 0 0;...
        0 0 0 0;...
        cos(theta)/m cos(theta)/m cos(theta)/m cos(theta)/m;...
        sin(theta)/m -sin(theta)/m sin(theta)/m -sin(theta)/m;...
        (w*cos(theta))/(2*I)-(l*sin(theta))/(2*I) -(w*cos(theta))/(2*I)+(l*sin(theta))/(2*I) -(w*cos(theta))/(2*I)+(l*sin(theta))/(2*I) (w*cos(theta))/(2*I)-(l*sin(theta))/(2*I)];
    F = eye(6)+A*dt;                                                    % Euler integration step
    G = B*dt;
    if k ~= N
        x(:,k+1) = F*x(:,k)+G*[T(1,k);T(2,k);T(3,k);T(4,k)];                % Euler solution
    end

end

function plot_results(N,dt,x,u,up,ui,ud,error)

    % Time matrix for plotting
    tt = 0:dt:(N-1)*dt;
    
    figure(); 
    plot_dir(x(1,:),x(2,:));
    xlabel('x-position [m]');
    ylabel('y-position [m]');
    title('Robot Position');
    grid on;
    
    figure();
    plot_dir(tt,rad2deg(x(3,:)));
    xlabel('time [s]');
    ylabel('angular position (heading) [deg]');
    title('Robot Heading vs. Time');
    grid on;
    
    figure();
    plot_dir(tt,x(4,:));
    xlabel('time [s]');
    ylabel('surge velocity [m/s]');
    title('Surge (Forward) Velocity vs. Time');
    grid on;
    
    figure();
    yyaxis left;                % (must have matlab 2016 to run)
    plot(tt,error(1,:));
    ylabel('heading error');
    yyaxis right;
    plot(tt,error(2,:));
    xlabel('time [s]');
    ylabel('velocity error');
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
    rosshutdown()
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

