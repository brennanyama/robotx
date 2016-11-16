function publish_motor_commands(u,MC,k,m_pub,m_msg)

    % Convert thrust to motor controller duty cycle
    MC(1,k) = u(1,k).*(100/55);
    MC(2,k) = u(2,k).*(100/55);
    MC(3,k) = u(3,k).*(100/60);
    MC(4,k) = u(4,k).*(100/60);
    
    m_msg(1).Data = round(MC(1, k), -1);
    m_msg(2).Data = round(MC(2, k), -1);
    m_msg(3).Data = round(MC(3, k), -1);
    m_msg(4).Data = round(MC(4, k), -1);
    
    % Publish to ROS
    send(m_pub(1),m_msg(1));
    send(m_pub(2),m_msg(2));
    send(m_pub(3),m_msg(3));
    send(m_pub(4),m_msg(4));
    
end
