function pwm = n2pwmR(newton)
%This function converts the output from the motion controller in Newtons,
%to a PWM signal that can be sent to the motor controllers.
    
    if ge(newton, 0)
        a = 0;
        b = 266.8920;
        c = 55;
        d = 100;
    elseif lt(newton, 0)
        a = -266.8920;
        b = 0;
        c = 0;
        d = 55;
    end
    
    pwm = round(((c+d) + (d-c)*((2*newton - (a+b))/(b-a)))/2);
    if pwm < 0
	pwm = 0;
    elseif pwm > 99
	pwm = 99;
    end
end
