function ros_start_end()
clear all;
close all;
clc;

rosinit;

for k = 1:1:50

    slightly_shift_right(k);
    
end

rosshutdown;

end
    
   function slightly_shift_right(k)
        disp('current iteration: ');
        disp(k);
    T=[40;60;60;40];
    chatterpub1 = rospublisher('/motor_q1', 'std_msgs/UInt16');
    chatterpub2 = rospublisher('/motor_q2', 'std_msgs/UInt16');
    chatterpub3 = rospublisher('/motor_q3', 'std_msgs/UInt16');
    chatterpub4 = rospublisher('/motor_q4', 'std_msgs/UInt16');


    chattermsg1 = rosmessage(chatterpub1);
    chattermsg2 = rosmessage(chatterpub2);
    chattermsg3 = rosmessage(chatterpub3);
    chattermsg4 = rosmessage(chatterpub4);
    
    chattermsg1.Data = T(1);
    chattermsg2.Data = T(2);
    chattermsg3.Data = T(3);
    chattermsg4.Data = T(4);
    
    send(chatterpub1, chattermsg1);
    send(chatterpub2, chattermsg2);
    send(chatterpub3, chattermsg3);
    send(chatterpub4, chattermsg4);    
   end
    