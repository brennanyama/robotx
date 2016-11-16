function [time_params,lumped_params,geometry_params,pid_gains,control_tolerances,control_maximums,x,u,up,ui,ud,MC,error,int,behavior] = sim_setup()

    % Time variables
    dt = .01;                       % simulation time step [s]
    tend = 500;                      % simulation end time [s]
    N = round(tend/dt);             % number of steps [ ]
    time_params = [dt,tend,N];      % output matrix
    
    % Model initial conditions
    x0(1) = 0;          % x-position [m]
    x0(2) = 0;          % y-position [m]
    x0(3) = 3*pi/2;         % psi (heading) [rad]
    x0(4) = 1;          % surge (forward) velocity [m/s]
    x0(5) = 0;          % sway (transverse) velocity [m/s]
    x0(6) = 0;          % yaw (heading) velocity [rad/s]
    
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
    kp_psi = 10;
    ki_psi = 0;
    kd_psi = 0;
    kp_vel = 20;
    ki_vel = 0;
    kd_vel = 10;
    pid_gains = [kp_psi,ki_psi,kd_psi,kp_vel,ki_vel,kd_vel];
    
    % Control tolerances
    GRT = 0.5;                                          % goal radius tolerance [m]
    PHT = deg2rad(15);                                  % path heading tolerance [rad]
    GHT = deg2rad(5);                                   % goal heading tolerance [rad]
    VT = 0.1;                                           % velocity tolerance [m/s]
    control_tolerances = [GRT,PHT,GHT,VT];              % output matrix
    
    % Control maximums
    u_max = 5;                                          % max surge velocity (control)
    v_max = 5;                                          % max sway velocity (control)
    T1_max = lb2N(55);                                  % max thrust (phyiscal)
    T2_max = lb2N(55);                                  % max thrust (physical)
    T3_max = lb2N(60);                                  % max thrust (physical)
    T4_max = lb2N(60);                                  % max thrust (phyiscal)
    reverse_thrust_ratio = 0.6;                         % max reverse thrust ratio
    control_maximums = [u_max,v_max,T1_max,T2_max,...   % output matrix
        T3_max,T4_max,reverse_thrust_ratio]; 
    
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
    up = zeros(4,N);            % thurster proportional gain [N]
    ui = zeros(4,N);            % thurster integral gain [N]
    ud = zeros(4,N);            % thurster derivative gain [N]
    u = zeros(4,N);             % thurster total gain [N]
    MC = zeros(4,N);            % motor command matrix [%]
    error = zeros(9,N);         % control variable error
    int = zeros(9,N);           % control variable integrator
    behavior = zeros(1,N);      % behavior tracking matrix       
end