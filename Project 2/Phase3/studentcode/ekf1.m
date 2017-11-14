function [X, Z] = ekf1(sensor, vic, p0,p1,p2,p3,p4)
% EKF1 Extended Kalman Filter with Vicon velocity as inputs
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
%   vic    - struct for storing vicon linear velocity in world frame and
%            angular velocity in body frame, fields include
%          - t: vicon timestamp
%          - vel = [vx; vy; vz; wx; wy; wz]
%   varargin - any variables you wish to pass into the function, could be
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              ekf1_handle = ...
%                  @(sensor, vic) ekf1(sensor, vic, your input arguments);
%
% OUTPUTS:
% X - nx1 state of the quadrotor, n should be greater or equal to 6
%     the state should be in the following order
%     [x; y; z; qw; qx; qy; qz; other states you use]
%     we will only take the first 7 rows of X
% OPTIONAL OUTPUTS:
% Z - mx1 measurement of your pose estimator, m shoulb be greater or equal to 7
%     the measurement should be in the following order
%     [x; y; z; qw; qx; qy; qz; other measurement you use]
%     note that this output is optional, it's here in case you want to log your
%     measurement
%% Initialization
persistent c X_k P_k old_vic
if isempty(c)
    if isempty(sensor.id) 
        Z=zeros(7,1);
        old_vic=vic;
        X=[];
        return;
        
    end
    [old_pos, q] = estimate_pose(sensor, p0,p1,p2,p3,p4);
    [att_old(1), att_old(2), att_old(3)] =RotToRPY_ZXY(inv(QuatToRot(q)));
    X_k=[old_pos;att_old'];
    X=[old_pos;RotToQuat((RPYtoRot_ZXY(att_old(1), att_old(2), att_old(3))))];
     %X=[];
    Z=[];
    old_vic=vic;
    P_k=eye(6);
    c=1;
    return;
end
%% Calulate Omega Estimated and measured
if (isempty(sensor.id))
    Xkhat=X_k;
    att=X_k(4:6);
else
    [pos, q] = estimate_pose(sensor,p0,p1,p2,p3,p4);
    [att(1),att(2),att(3)]=RotToRPY_ZXY((QuatToRot(q)));
    Xkhat=[pos;att']; 
end

G=[cos(att(2)) 0 -cos(att(1))*sin(att(2));
   0          1             sin(att(1)); 
   sin(att(2)) 0 cos(att(1))*cos(att(2))];
qdot=G\vic.vel(4:6);
Ukhat=[vic.vel(1:3);qdot]; 
dt=vic.t-old_vic.t;

%% Process Update
xk_1=X_k; 
pk_1=P_k;
R=eye(6);
A=eye(6);
B=eye(6).*dt;

Xk=A*xk_1+B*Ukhat;
Pk=A*pk_1*transpose(A);

kg=Pk./(Pk+R);
kg(isnan(kg))=0;

X_k=Xk+kg*(Xkhat-Xk);
P_k=(eye(6)-kg)*Pk;



%% Measuremnet Update 

old_vic=vic;

X = [X_k(1:3);RotToQuat((RPYtoRot_ZXY(X_k(4),X_k(5),X_k(6))))];
Z = zeros(7,1);

end
