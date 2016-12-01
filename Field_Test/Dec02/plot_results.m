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