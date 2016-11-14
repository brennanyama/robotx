function plot_results(time_params,goal_vars,control_tolerances,x,u,up,ui,ud,error)

    % Time matrix for plotting
    tt = 0:time_params(1):(time_params(3)-1)*time_params(1);
    
    figure(); 
    th = 0:pi/50:2*pi;
    xunit = control_tolerances(1) * cos(th) + goal_vars(1);
    yunit = control_tolerances(1) * sin(th) + goal_vars(2);
    plot(x(1,:),x(2,:),'b.-',goal_vars(1),goal_vars(2),'r*',xunit, yunit,'r');
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
    plot(tt,x(4,:),'b.-')
    xlabel('time [s]');
    ylabel('surge velocity [m/s]');
    title('Robot Surge Velocity vs. Time');
    grid on;
    
    figure();
    yyaxis left;                % (must have matlab 2016 to run)
    plot(tt,error(4,:));
    ylabel('distance error [m]');
    yyaxis right;
    plot(tt,rad2deg(error(5,:)));
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