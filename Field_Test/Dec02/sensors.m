function meas = sensors(y,meas,k,dt,time_params,snr,rr)

    if k ~= time_params(3)
        % Position
        if mod(k*dt,1/rr(1)) == 0                           % update sensor reading at rr
            meas(1:3,k+1) = awgn(y(1:3,k+1),snr(1));        % add noise
        else
            meas(1:3,k+1) = meas(1:3,k);
        end

        % Velocity
        if mod(k*dt,1/rr(2)) == 0                           % update sensor reading at rr
            meas(4:6,k+1) = awgn(y(4:6,k+1),snr(2));        % add noise
        else
            meas(4:6,k+1) = meas(4:6,k);
        end

        % Acceleration
        if mod(k*dt,1/rr(3)) == 0                           % update sensor reading at rr
            meas(7:9,k+1) = awgn(y(7:9,k+1),snr(3));        % add noise
        else
            meas(7:9,k+1) = meas(7:9,k);
        end
    end
    
end