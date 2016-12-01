function [time_params,lumped_params,geometry_params,pid_gains,control_tolerances,control_maximums,x,y,snr,rr,meas,u,up,ui,ud,error,int,behavior] = sim_setup()

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
    kp_pos = 10
    ki_pos = 0;
    kd_pos = 2
    kp_psi = 10;
    ki_psi = 0     %10,0,0,40,0.5,5,5,0,2
    kd_psi = 2
    kp_vel = 10
    ki_vel = 0;
    kd_vel = 2
    pid_gains = [kp_pos,ki_pos,kd_pos,...
        kp_psi,ki_psi,kd_psi,...
        kp_vel,ki_vel,kd_vel];
    
    % Control tolerances
    AR = 5                                           % approach radius [m]
    GR = 0.5;                                           % goal radius [m]
    PHT = deg2rad(10);                                  % path heading tolerance [rad]
    GHT = deg2rad(5);                                   % goal heading tolerance [rad]
    VT = 0.1;                                           % velocity tolerance [m/s]
    control_tolerances = [AR,GR,PHT,GHT,VT];            % output matrix
    
    % Initialize state and output matrix
    x = zeros(6,N);     % NEED TO CHANGE TO TAKE DATA FROM STATE ESTIMATION
    x(1,1) = 0;         % x-position initial condition
    x(2,1) = 0;         % y-position initial condition
    x(3,1) = 0;         % psi (heading) initial condition
    x(4,1) = 1;         % surge (forward) velocity initial condition
    x(5,1) = 1;         % sway (transverse) velocity initial condition
    x(6,1) = 1;         % psi (heading) velocity initial condition
    y = zeros(9,N);
    
    % Sensor variables
    snr_pos = 0.5;                  % signal to noise ratio [ ]
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
    int = zeros(10,N);      % control variable integrator
    error = zeros(10,N);    % error tracking matrix
    behavior = zeros(1,N);  % behavior tracking matrix

end
