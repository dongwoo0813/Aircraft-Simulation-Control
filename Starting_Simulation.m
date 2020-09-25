%Initialize constants for the RCAM simulation
clear; clc; close all

%% Define Constants

% Initially airplane going 165 knots at pitch of 5.73 degrees
x0 = [85;
      0;
      0;
      0;
      0;
      0;
      0;
      0.1;
      0]; 

u = [0;
    -0.1;
    0;
    0.08;
    0.08];  % u2 initially at -5.73 degrees
                              % u4 and u5 at minimum initially
                              
TF = 60;                      %sec, simulation time


%% Reference

x_ref = [85;
        0;
        0;
        0;
        0;
        0;
        0;
        0;
        0];

% Kp = eye(9);
% Ki = 0.001*eye(9);
% Kd = eye(9);



%% Run the model
out = sim('RCAM_Simulation.slx')

%%
figure(1)

subplot(3,3,1)
plot(out.X.Time, out.X.Data(:,1))
title('velocity in x-axis')
xlabel('time, second')
ylabel('u, m/s')

subplot(3,3,2)
plot(out.X.Time, out.X.Data(:,2))
title('velocity in y-axis')
xlabel('time, second')
ylabel('v, m/s')

subplot(3,3,3)
plot(out.X.Time, out.X.Data(:,3))
title('velocity in z-axis')
xlabel('time, second')
ylabel('w, m/s')

subplot(3,3,4)
plot(out.X.Time, out.X.Data(:,4))
title('angular velocity about x-axis')
xlabel('time, second')
ylabel('p, rad/s')

subplot(3,3,5)
plot(out.X.Time, out.X.Data(:,5))
title('angular velocity about y-axis')
xlabel('time, second')
ylabel('q, rad/s')

subplot(3,3,6)
plot(out.X.Time, out.X.Data(:,6))
title('angular velocity about z-axis')
xlabel('time, second')
ylabel('r, rad/s')

subplot(3,3,7)
plot(out.X.Time, out.X.Data(:,7))
title('Roll Euler Angle')
xlabel('time, second')
ylabel('phi, rad')

subplot(3,3,8)
plot(out.X.Time, out.X.Data(:,8))
title('Pitch Euler Angle')
xlabel('time, second')
ylabel('theta, rad')

subplot(3,3,9)
plot(out.X.Time, out.X.Data(:,9))
title('Yaw Euler Angle')
xlabel('time, second')
ylabel('psi, rad')


% figure(2)
% subplot(5,1,1)
% plot(out.U.Time, out.U.Data(:,1))
% title('Aileron Deflection')
% xlabel('time, second')
% ylabel('\delta_A, rad')
% 
% subplot(5,1,2)
% plot(out.U.Time, out.U.Data(:,2))
% title('Horizontal Stabilizer Deflection')
% xlabel('time, second')
% ylabel('\delta_S, rad')
% 
% subplot(5,1,3)
% plot(out.U.Time, out.U.Data(:,3))
% title('Rudder Deflection')
% xlabel('time, second')
% ylabel('\delta_R, rad')
% 
% subplot(5,1,4)
% plot(out.U.Time, out.U.Data(:,4))
% title('Throttle 1')
% xlabel('time, second')
% ylabel('\delta_th1, rad')
% 
% subplot(5,1,5)
% plot(out.U.Time, out.U.Data(:,5))
% title('Throttle 2')
% xlabel('time, second')
% ylabel('\delta_th2, rad')














