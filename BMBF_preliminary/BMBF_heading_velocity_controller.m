function heading_velocity_controller()

    % Controller setup
    [v0,psi0,k,dt] = controller_setup();
    
    % Motor pin setup
    m_pin = m_pin_assign();

    % Output matrices
    v = zeros(1E6,1);       % measured velocity
    psi = zeros(1E6,1);     % measured heading
    v(1) = v0;
    psi(1) = psi0;

    % Goal variables
    goal = false;
    v_goal = 3;         % desired velocity [m/s]
    psi_goal = 5;       % desired heading [deg from north]
    
    % Instantiate controller variables
    v_last = 0;
    psi_last = 0;
    v_int = 0;
    psi_int = 0;
    e_v_last = 0;
    e_psi_last = 0;
    dt = 0.1;
    
    % Controller
    n = 2;
    while goal == false
        
        % Check for goal
        
        % Calculate gains
        [u_FQ1,u_FQ2,u_FQ3,u_FQ4,v_int,psi_int] = PID_gains(v_goal,psi_goal,v,psi,v_int,psi_int,v_last,psi_last,dt)
        
        % Send drive command to motor
        
        
        n = n+1;
    end

end

function [v0,psi0,k,dt] = controller_setup()
    % Setup controller

    % Initial vars
    v0 = 0;     % initial velocity (on the real robot, this is measured!)
    psi0 = 0;   % initial heading (on the real robot, this is measured!)

    % Threshold values
    vt = 0.5;   % success threshold velocity [m/s]
    psit = 5;   % success threshold heading [deg]

    % Controller gains
    kp = 10;
    ki = 2;
    kd = 1;
    k = [kp,ki,kd];

    % Time step
    dt = 0.1;

end

function m_pin = m_pin_assign()
    mQ1_pin = 8;
    mQ2_pin = 9;
    mQ3_pin = 10;
    mQ4_pin = 11;
end

function [u_FQ1,u_FQ2,u_FQ3,u_FQ4,v_int,psi_int] = PID_gains(v_goal,psi_goal,v,psi,v_int,psi_int,v_last,psi_last,dt)

    % Calculate error
    e_v = v_goal-v;
    e_psi = psi_goal-psi;
    
    % Proportional gain
    up_FQ1 = kp*(e_v+e_psi);
    up_FQ2 = kp*(e_v-e_psi);
    up_FQ3 = kp*(e_v+e_psi);
    up_FQ4 = kp*(e_v-e_psi);    
    
    % Integral gain
    ui_FQ1 = ki*(v_int+psi_int);
    ui_FQ2 = ki*(v_int-psi_int);
    ui_FQ3 = ki*(v_int+psi_int);
    ui_FQ4 = ki*(v_int-psi_int);
    v_int = v_int+e_v;
    psi_int = psi_int+e_psi;
           
    % Derivative gain ... needs input of last error
    ud_FQ1 = kd*((v-v_last)/dt+(psi-psi_last)/dt);
    ud_FQ2 = kd*((v-v_last)/dt-(psi-psi_last)/dt);
    ud_FQ3 = kd*((v-v_last)/dt+(psi-psi_last)/dt);
    ud_FQ4 = kd*((v-v_last)/dt-(psi-psi_last)/dt);
        
    % Gain vectors
    u_FQ1 = up_FQ1+ui_FQ1+ud_FQ1;
    u_FQ2 = up_FQ2+ui_FQ1+ud_FQ2;
    u_FQ3 = up_FQ3+ui_FQ1+ud_FQ3;
    u_FQ4 = up_FQ4+ui_FQ1+ud_FQ4;  
    
        
end
