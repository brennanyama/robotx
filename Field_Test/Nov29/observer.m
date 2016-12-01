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
end
