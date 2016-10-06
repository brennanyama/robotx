% still work in progress... ask brennan if you have questions.

function heading_velocity_controller_and_simulation()

    % Clear workspace
    clear all;
    close all;
    clc;
    
    % Simulation parameters
    [dt,tend,N] = sim_params();

    % System inputs
    [x0,m,I,bu,bv,bpsi,theta,w,l,K] = input_vars();
    
    % Create state vector
    x = state_vector(N,x0);
    
    % Set goal variables
    [v_goal,psi_goal] = goal_vars();
    
    % Set input matrix
    FQ = [0,0,0,0];
    
    % Tracking matrics
    T = zeros(4,N);     % thruster output [N] (max 245 N)
    up = zeros(4,N);    % thurster proportional gain [N]
    ui = zeros(4,N);    % thurster integral gain [N]
    ud = zeros(4,N);    % thurster derivative gain [N]
    u = zeros(4,N);     % thurster total gain [N]
    error = zeros(2,N); % control variable error
    int = zeros(2,N);   % control variable integrator
          
    % Start controller ... all this should get functioned once working
    % properly...
    for k = 2:1:N-1
                
        % Calculate error
        error(1,k) = psi_goal-x(3,k);       % heading error
        error(2,k) = v_goal-x(4,k);         % velocity error
        
        % Proportional gain (increases with large errors)
        up(1,k) = K(1)*(error(2,k)+error(1,k));
        up(2,k) = K(1)*(error(2,k)-error(1,k));
        up(3,k) = K(1)*(error(2,k)+error(1,k));
        up(4,k) = K(1)*(error(2,k)-error(1,k));    

        % Integral gain (increases with large change in error)
        ui(1,k) = K(2)*(int(2,k)+int(1,k));
        ui(2,k) = K(2)*(int(2,k)-int(1,k));
        ui(3,k) = K(2)*(int(2,k)+int(1,k));
        ui(4,k) = K(2)*(int(2,k)-int(1,k));
        int(1,k+1) = int(1,k)+error(1,k);       % increment integrator
        int(2,k+1) = int(2,k)+error(2,k);
        
        % Derivative gain (increases with large change in state)
        u(1,k) = K(3)*((x(4,k)-x(4,k-1))/dt+(x(3,k)-x(3,k-1))/dt);
        u(2,k) = K(3)*((x(4,k)-x(4,k-1))/dt-(x(3,k)-x(3,k-1))/dt);
        u(3,k) = K(3)*((x(4,k)-x(4,k-1))/dt+(x(3,k)-x(3,k-1))/dt);
        u(4,k) = K(3)*((x(4,k)-x(4,k-1))/dt-(x(3,k)-x(3,k-1))/dt);

        % Total gain vector
        u(1,k) = up(1,k)+ui(1,k)+ud(1,k);
        u(2,k) = up(2,k)+ui(2,k)+ud(2,k);
        u(3,k) = up(3,k)+ui(3,k)+ud(3,k);
        u(4,k) = up(4,k)+ui(4,k)+ud(4,k);
                
        % Iterate inputs
        T(1,k) = T(1,k)+u(1,k);
        T(2,k) = T(2,k)+u(2,k);
        T(3,k) = T(3,k)+u(2,k);
        T(4,k) = T(4,k)+u(4,k);
        
        % Plant simulation 
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
        x(:,k+1) = F*x(:,k)+G*[T(1,k);T(2,k);T(3,k);T(4,k)];                % Euler solution
    end
    
% % %     % Simulate plant (via Euler integration)
% % %     x = plant(N,dt,m,I,bu,bv,bpsi,theta,w,l,x,FQ);
    
    %% Plots

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
    plot_dir(tt,rad2deg(x(4,:)));
    xlabel('time [s]');
    ylabel('surge velocity [m/s]');
    title('Surge (Forward) Velocity vs. Time');
    grid on;
    
    % Assign whatever to workspace
    assignin('base','x',x);
    
end

function [dt,tend,N] = sim_params()

    % Time variables
    dt = .01;           % simulation time step [s]
    tend = 100;         % simulation end time [s]
    N = round(tend/dt); % number of steps [ ]

end

function [x0,m,I,bu,bv,bpsi,theta,w,l,k] = input_vars()

    % Initial conditions
    x0(1) = 0;          % x-position [m]
    x0(2) = 0;          % y-position [m]
    x0(3) = pi/2;       % psi (heading) [rad]
    x0(4) = 0;          % surge (forward) velocity [m/s]
    x0(5) = 0;          % sway (transverse) velocity [m/s]
    x0(6) = x0(3);      % psi (heading) [rad]
    
    % Model parameters
    m = 400;                 % mass [kg]
    I = 300;                 % inertia [kg*m^2]
    bu = 40;                 % surge (longitudinal) drag [N*s/m]
    bv = 200;                % sway (transverse) drag [N*s/m]
    bpsi = 200;               % yaw (heading) drag [N*s/(m*rad)]
    
    theta = deg2rad(30);    % thruster mount angle 
    w = 2;                  % width between thrusters [m]
    l = 5;                  % length between thrusters [m]
    
    % Controller gains
    kp = 1;
    ki = 1;
    kd = 1;
    k = [kp ki kd];
    
end

function x = state_vector(N,x0)
    
    % Create vector
    x = zeros(6,N);
    
    % Insert initial conditions
    x(1,1) = x0(1);     % x-position 
    x(2,1) = x0(2);     % y-position
    x(3,1) = x0(3);     % psi (heading)
    x(4,1) = x0(4);     % surge (forward) velocity
    x(5,1) = x0(5);     % sway (transverse) velocity
    x(6,1) = x0(6);     % psi (heading)
    
end

function [v_goal,psi_goal] = goal_vars()
   
    v_goal = 5;                 % goal velocity [m/s]
    psi_goal = deg2rad(45);    % goal heading [rad]

end

function x = plant(N,dt,m,I,bu,bv,bpsi,theta,w,l,x,FQ) 

    for k = 1:1:N-1
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
        x(:,k+1) = F*x(:,k)+B*dt*[FQ(1);FQ(2);FQ(3);FQ(4)];                 % Euler solution
    end

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
