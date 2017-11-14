function [X, Z] = ekf2(sensor, p0,p1,p2,p3,p4)
% EKF2 Extended Kalman Filter with IMU as inputs
%
% INPUTS:
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: sensor timestamp
%          - rpy, omg, acc: imu readings
%          - img: uint8, 240x376 grayscale image
%          - id: 1xn ids of detected tags
%          - p0, p1, p2, p3, p4: 2xn pixel position of center and
%                                four corners of detected tags
%            Y
%            ^ P3 == P2
%            | || P0 ||
%            | P4 == P1
%            o---------> X
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor) ekf2(sensor, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 9
%     the state should be in the following order
%     [x; y; z; vx; vy; vz; qw; qx; qy; qz; other states you use]
%     we will only take the first 10 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 6
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
persistent c X_k P_k sensor_old
%% Inialization
if isempty(c)
    if isempty(sensor.id)
        X=[0,0,0,0,0,0,0,0,0,0];
        Z=[];
        sensor_old=sensor;
        return;
    end
    [old_pos, q] = estimate_pose(sensor, p0,p1,p2,p3,p4);
    [att_old(1), att_old(2), att_old(3)] =RotToRPY_ZXY(inv(QuatToRot(q)));
    X_k=[old_pos;0;0;0;att_old'];
    X=[old_pos;0;0;0;RotToQuat(RPYtoRot_ZXY(att_old(1), att_old(2), att_old(3)))];
  % X=X_k;
    Z=[];
    P_k=eye(9);
    c=1;
    sensor_old=sensor;
    return;
end
dt=sensor.t-sensor_old.t;
 %% aa
 vel=[];
 if isempty(sensor.id)
     Xkhat=X_k;
    att=X_k(7:9);
    else
    [pos, q] = estimate_pose(sensor,p0,p1,p2,p3,p4);
    [att(1),att(2),att(3)]=RotToRPY_ZXY(inv(QuatToRot(q)));
    [vel, ~]=estimate_vel(sensor,p0,p1,p2,p3,p4);
    Xkhat=[pos;vel;att'];
 end
 G=[cos(att(2)) 0 -cos(att(1))*sin(att(2));
   0          1             sin(att(1)); 
   sin(att(2)) 0 cos(att(1))*cos(att(2))];

sensor.acc=RPYtoRot_ZXY(att(1),att(2),att(3))\sensor.acc;
sensor.acc(3)=sensor.acc(3)-9.81;
if isempty(vel)
    vel=[0;0;0];
end
Ukhat=[vel;sensor.acc;G\sensor.omg];
%% bb
xk_1=X_k; 
pk_1=P_k;
R=eye(9);
A=eye(9);
B=eye(9).*dt;
Q=eye(9);

Xk=A*xk_1+B*Ukhat;
Pk=A*pk_1*transpose(A)+Q;

kg=Pk./(Pk+R);
kg(isnan(kg))=0;

X_k=Xk+kg*(Xkhat-Xk);
P_k=(eye(9)-kg)*Pk;


X = [X_k(1:6);RotToQuat((RPYtoRot_ZXY(X_k(7),X_k(8),X_k(9))))];
Z = zeros(7,1);
%% update
sensor_old=sensor;
end
