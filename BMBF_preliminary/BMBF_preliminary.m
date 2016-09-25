function BMBF_preliminary

    % This code is making the assumption that the LIDAR unit will not be available

    % Get location information (current position (x,y), and current heading (psi))
    
    % Engage target search behavior
        % Look through camera
            % If you can see the targets, move on
            % If you can't see the targets, spin
    
    % Engage target follow behavior
        % Drive forward at constant velocity
    
        % Calculate heading relative to the mean (middle) of the targets, dpsi.  This represents your error function.
        % Update controller gains as a result of heading error

end
