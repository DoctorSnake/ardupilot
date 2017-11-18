// directory = "/home/johndoe/Documents/Drone_VTAIL/AutoPilot/ardupilot_3.2.1/QuadControl/logs";

//mat = csvRead('/home/johndoe/Documents/Drone_VTAIL/AutoPilot/ardupilot_3.2.1/QuadControl/logs/quad_log_1.csv', ',');
//mat = csvRead('/home/johndoe/Documents/Drone_VTAIL/AutoPilot/ardupilot_3.2.1/QuadControl/logs/quad_log_2.csv', ',');
//mat = csvRead('/home/johndoe/Documents/Drone_VTAIL/AutoPilot/ardupilot_3.2.1/QuadControl/logs/quad_log_3.csv', ',');
mat = csvRead('/home/johndoe/Documents/Drone_VTAIL/AutoPilot/ardupilot_3.2.1/QuadControl/logs/quad_log_4.csv', ',');

roll_input = mat(:,1);
pitch_input = mat(:,2);
throttle_input = mat(:,3);
yaw_input = mat(:,4);

roll = mat(:,5);
pitch = mat(:,6);
yaw = mat(:,7);

mot_fl = mat(:,8);
mot_bl = mat(:,9);
mot_fr = mat(:,10);
mot_br = mat(:,11);

time =  mat(:,12);

rcroll = mat(:,13);
rcpitch = mat(:,14);
rcthrottle = mat(:,15);
rcyaw = mat(:,16);

roll_stab_out = mat(:,17);
pitch_stab_out = mat(:,18);
yaw_stab_out = mat(:,19);


roll_out = mat(:,20);
pitch_out = mat(:,21);
yaw_out = mat(:,22);

//=======================================================
figure(1)
subplot(2,2,1)
set(gca(),"auto_clear","off")
plot(time,roll_input,'b')
plot(time,roll,'r')
set(gca(),"grid",[1 1])

subplot(2,2,2)
set(gca(),"auto_clear","off")
plot(time,pitch_input,'b')
plot(time,pitch, 'r')
set(gca(),"grid",[1 1])

subplot(2,2,3)
set(gca(),"auto_clear","off")
plot(time,yaw_input,'b')
plot(time,yaw,'r')
set(gca(),"grid",[1 1])

subplot(2,2,4)
plot(time,rcthrottle)


//=======================================================
figure(2)
subplot(2,2,1)
plot(time,rcpitch)
title('pitch radio')
set(gca(),"grid",[1 1])

subplot(2,2,2)
plot(time,rcroll)
title('roll  radio')
set(gca(),"grid",[1 1])

subplot(2,2,3)
plot(time,rcyaw)
title('yaw  radio')
set(gca(),"grid",[1 1])

subplot(2,2,4)
plot(time,rcthrottle)
title('throttle  radio')

//=======================================================
figure(3)
subplot(3,1,1)
plot(time,roll)

subplot(3,1,2)
plot(time,pitch)

subplot(3,1,3)
plot(time,yaw)



figure(4)
subplot(2,2,1)
plot(time,mot_fr)
title('motor front right')
set(gca(),"grid",[1 1])

subplot(2,2,2)
plot(time,mot_bl)
title('motor back left')
set(gca(),"grid",[1 1])

subplot(2,2,3)
plot(time,mot_fl)
title('motor front left')
set(gca(),"grid",[1 1])

subplot(2,2,4)
plot(time,mot_br)
title('motor back right')
set(gca(),"grid",[1 1])


figure(5)
subplot(2,2,1)
plot(time,pitch_input - pitch)
title('pitch error')
set(gca(),"grid",[1 1])

subplot(2,2,2)
plot(time,roll_input - roll)
title('roll error')
set(gca(),"grid",[1 1])

subplot(2,2,3)
plot(time,yaw_input - yaw)
title('yaw error')
set(gca(),"grid",[1 1])

subplot(2,2,4)
plot(time,rcthrottle)
title('throttle')
set(gca(),"grid",[1 1])

// Informations à rajouter
// pitch_output, roll_output, yaw_output
// pitch_stab_output
// roll_stab_output
// yaw_stab_output
// gyroPitch
// gyroRoll
// gyroYaw
