function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================
acc_des=[0,0,0];
%hover control gains
kp=[4,4,200];
kd=[8,8,50];
g=9.8;
% Atitude Control gains
kp_euler=[100,100,200];
kd_euler=[50,50,50];

I = [1.43e-5,   0,          0; % inertial tensor in m^2 kg
     0,         1.43e-5,    0;
     0,         0,          2.89e-5];

% Desired roll, pitch and yaw
acc_des(1)=qd{qn}.acc_des(1)+kd(1)*(qd{qn}.vel_des(1)-qd{qn}.vel(1))+kp(1)*(qd{qn}.pos_des(1)-qd{qn}.pos(1));
acc_des(2)=qd{qn}.acc_des(2)+kd(2)*(qd{qn}.vel_des(2)-qd{qn}.vel(2))+kp(2)*(qd{qn}.pos_des(2)-qd{qn}.pos(2));

phi_des =(1/g)*(acc_des(1)*sin(qd{qn}.yaw_des)-acc_des(2)*cos(qd{qn}.yaw_des));
theta_des =(1/g)*(acc_des(1)*cos(qd{qn}.yaw_des)+acc_des(2)*sin(qd{qn}.yaw_des));
psi_des = qd{qn}.yaw_des;

p_des=0;
q_des=0;
r_des=qd{qn}.yawdot_des;

% Thurst
acc_des(3)=qd{qn}.acc_des(3)+kd(3)*(qd{qn}.vel_des(3)-qd{qn}.vel(3))+kp(3)*(qd{qn}.pos_des(3)-qd{qn}.pos(3));
F    = params.mass*g+params.mass*acc_des(3);

% Moment

M    = I*[kp_euler(1)*(phi_des-qd{qn}.euler(1))+kd_euler(1)*(p_des-qd{qn}.omega(1)),...
        kp_euler(2)*(theta_des-qd{qn}.euler(2))+kd_euler(2)*(q_des-qd{qn}.omega(2)),...
        kp_euler(3)*(psi_des-qd{qn}.euler(3))+kd_euler(3)*(r_des-qd{qn}.omega(3))]'; % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
