function [F, M, trpy, drpy] = controller(qd, t, qn, params)
% CONTROLLER quadrotor controller
% The current states are:
% qd{qn}.pos, qd{qn}.vel, qd{qn}.euler = [roll;pitch;yaw], qd{qn}.omega
% The desired states are:
% qd{qn}.pos_des, qd{qn}.vel_des, qd{qn}.acc_des, qd{qn}.yaw_des, qd{qn}.yawdot_des
% Using these current and desired states, you have to compute the desired controls

% =================== Your code goes here ===================

% Desired roll, pitch and yaw
Desired_state=qdToState;
phi_des = qd{qn}.euler(1);
theta_des = qd{qn}.euler(2);
psi_des = qd{qn}.euler(3);
F1=params.forceconstant*omega_1^2;
F2=params.forceconstant*omega_2^2;
F3=params.forceconstant*omega_3^2;
F4=params.forceconstant*omega_4^2;
A_R_B=RPYtoRot_ZXY(phi_des,theta_des,psi_des);
% [cos(psi_des)*cos(theta_des)-sin(phi_des)*sin(psi_des)*sin(theta_des), -cos(phi_des)*sin(psi_des), cos(psi_des)*sin(theta_des)+cos(theta_des)*sin(phi_des)*sin(psi_des);
%        cos(theta_des)*sin(psi_des)+cos(psi_des)*sin(phi_des)*sin(theta_des), cos(psi_des)*cos(phi_des),sin(psi_des)*sin(theta_des)-cos(psi_des)*cos(theta_des)*sin(phi_des);
%        -cos(phi_des)*sin(theta_des),sin(phi_des),cos(phi_des)* cos(theta_des);];
gamma=params.torqueconstant/params.forceconstant;
Jw=[cos(theta_des),0,-cos(phi_des)*sin(theta_des);
              0,1,sin(phi_des);
              sin(theta),0,cos(phi_des)*cos(theta_des);];
Ang_vel =Jw*[Desired_state(11),Desired_state(12), Desired State(13)];          
r_accel_inertialframe= [0,0,g]'+A_R_B*[0,0,F1+F2+F3+F4]';
u1=F1+F2+F3+F4;
u2=[0,params.arm_length,0,-params.arm_length;-params.arm_length,0,params.arm_length,0;gamma,-gamma,gamma,-gamma;]*[0,0,F1+F2+F3+F4]';




% Thurst
F    = 0;

% Moment
M    = zeros(3,1); % You should fill this in
% =================== Your code ends here ===================

% Output trpy and drpy as in hardware
trpy = [F, phi_des, theta_des, psi_des];
drpy = [0, 0,       0,         0];

end
