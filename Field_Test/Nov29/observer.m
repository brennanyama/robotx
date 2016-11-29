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
