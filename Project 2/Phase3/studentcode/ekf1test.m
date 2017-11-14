function [X, Z] = ekf1test(sensor, vic, p0,p1,p2,p3,p4)
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

persistent flag old_t Xk Pk

if isempty(flag)
    if~(isempty(sensor.id))
        old_t=sensor.t;
        [pos, q] = estimate_pose(sensor,p0,p1,p2,p3,p4);
        [phi,theta,psi]=RotToRPY_ZXY(inv(QuatToRot(q)));
        Xk=[pos;phi;theta;psi]; Pk=eye(6);
        flag=1;
    else
        old_t=sensor.t;
        Xk=[]; Pk=eye(9);
    end
    X=[pos;q];Z=[];
    
    return;
end

%% CURRENT STATE MEASUREMENT%%

if(isempty(sensor.id) && isempty(vic))
    X=[];   Z=[];
    return;
end

if (isempty(sensor.id))
    Y=Xk;
    phi=Xk(4);  theta=Xk(5);
else
    [pos, q] = estimate_pose(sensor,p0,p1,p2,p3,p4);

    [phi,theta,psi]=RotToRPY_ZXY(inv(QuatToRot(q)));

    Y=[pos;phi;theta;psi]; 
end

G=[cos(theta) 0 -cos(phi)*sin(theta);
   0          1             sin(phi); 
   sin(theta) 0 cos(phi)*cos(theta)];

vicvel=vic.vel(1:3);    vicomg=vic.vel(4:6);
q_dot=G\vicomg;
Uk=[vicvel;q_dot]; 
delT=vic.t-old_t; 

%% NEW STATE PREDICTION %%

Xk_1=Xk;    Pk_1=Pk;

R=eye(6);   

A=eye(6);

B=eye(6)*delT;  

%Wk=zeros(6,1); Qk=zeros(6,1);

Xkp=A*Xk_1+B*Uk;%+Wk;     
Pkp=A*Pk_1*transpose(A);%+Qk;

%% UPDATING AND KALMAN GAIN %%

KG=Pkp./(Pkp+R);

KG(isnan(KG))=0;

Xk=Xkp+KG*(Y-Xkp);

Pk=(eye(6)-KG)*Pkp;

old_t=vic.t;

dummy=RotToQuat(RPYtoRot_ZXY(Xk(4),Xk(5),Xk(6)));

X = [Xk(1:3);dummy];
Z = zeros(7,1);

end