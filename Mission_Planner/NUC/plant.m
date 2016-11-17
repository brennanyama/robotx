function x = plant(k,time_params,x,u,lumped_params,geometry_params)

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
        -sin(theta)/m sin(theta)/m -sin(theta)/m sin(theta)/m;...
        (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I) -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I) -(w*cos(theta))/(2*I)-(l*sin(theta))/(2*I) (w*cos(theta))/(2*I)+(l*sin(theta))/(2*I)];
    F = eye(6)+A*time_params(1);                                            % Euler integration step
    G = B*time_params(1);
    if k ~= time_params(3)
        x(:,k+1) = F*x(:,k)+G*[u(1,k);u(2,k);u(3,k);u(4,k)];                % Euler solution
    end

end