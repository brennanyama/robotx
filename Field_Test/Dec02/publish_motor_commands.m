function publish_motor_commands(m_pub,m_msg)
    % Publish to ROS
    send(m_pub(1),m_msg(1));
    send(m_pub(2),m_msg(2));
    send(m_pub(3),m_msg(3));
    send(m_pub(4),m_msg(4));
end
