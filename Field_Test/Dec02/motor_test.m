Q1pub = rospublisher('/motor_q1', 'std_msgs/UInt16');
Q2pub = rospublisher('/motor_q2', 'std_msgs/UInt16');
Q3pub = rospublisher('/motor_q3', 'std_msgs/UInt16');
Q4pub = rospublisher('/motor_q4', 'std_msgs/UInt16');

Q1msg = rosmessage(Q1pub);
Q2msg = rosmessage(Q2pub);
Q3msg = rosmessage(Q3pub);
Q4msg = rosmessage(Q4pub);

k = 1;
j = 1;
exit = false;
valid = false;

while (~valid)
	inc = input('Desired increment [int]: ');
	valid = true;
	if(isequal(inc, 69))
		disp('Ew, you perv.');
		valid = false;
	end
end

while (~exit)
	modifier = mod(k*inc, 100);
	Q1msg.Data = modifier;
	Q2msg.Data = modifier;
	Q3msg.Data = modifier;
	Q4msg.Data = modifier;
	
	disp([Q1msg.Data, Q2msg.Data, Q3msg.Data, Q4msg.Data]);

	send(Q1pub, Q1msg);
	send(Q2pub, Q2msg);
	send(Q3pub, Q3msg);
	send(Q4pub, Q4msg);
	k = k+1;
	if gt(k*inc, j*100)
		char = input('Continue? (Yes/No) ');
		if or(isequal(char, 'Yes'), isequal(char, 'yes'))
			exit = false;
		elseif or(isequal(char, 'No'), isequal(char, 'no'))
			exit = true;
		end
		j = j+1;
	end	
	pause(5);
end	
