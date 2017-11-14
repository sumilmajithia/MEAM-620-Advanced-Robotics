function [vel, omg] = estimate_vel(sensor,p0,p1,p2,p3,p4)
%ESTIMATE_VEL 6DOF velocity estimator
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
%          - t: timestamp
%          - rpy, omg, acc: imu readings, you should not use these in this phase
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
%              a data structure to represent the map or camera parameters,T%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor

%% Initializing 
persistent c PointTracker POINTS t delta_t_old img_old Velocity_old u
if isempty(sensor.id)
    vel=[];
    omg=[];
    return;
end
if isempty(c)
    corners=detectFASTFeatures(sensor.img);
    PointTracker = vision.PointTracker;%('MaxBidirectionalError',1);
    %corners_location=corners.selectStrongest(500).Location;
    initialize(PointTracker,corners.Location,sensor.img); 
    POINTS=corners.Location;
    t=sensor.t;
    vel=[0 0 0]';
    omg=[0 0 0]';
    c=1;
    delta_t_old=0;
    img_old=sensor.img;
    Velocity_old=[0 0 0 0 0 0];
    return;
end

%% update tarcker for new pointss

points=detectFASTFeatures(img_old);
PointTracker = vision.PointTracker;%('MaxBidirectionalError',1);
%points_location=points.selectStrongest(500).Location;
initialize(PointTracker,points.Location,img_old); 
POINTS=points.Location;
if isempty(u)
alpha=1;

else
    alpha=0.1;
end
u=1;
delta_t=alpha*(sensor.t-t)+(1-alpha)*delta_t_old;
[points,~,~]= step(PointTracker,sensor.img) ;

%% findind Z
K=[311.0520 0        201.8724; ...
0         311.3885 113.6210; ...
0         0        1];

p=[points,ones(size(points,1),1)];
P=[POINTS,ones(size(points,1),1)];
A=[];
B=[];

% those points in camera frame
for i=1: size(points,1)
A=[A;((K)\p(i,:)')'];
B=[B;((K)\P(i,:)')'];
end

[T,R,Hcamtoimu]=estimate_pose(sensor,p0,p1,p2,p3,p4);

Z=[];
for i=1: size(A,1)
Z=[Z;-T(3)/(R(3,1)*A(i,1)+R(3,2)*A(i,2)+R(3,3))];
end

%% calculating actual flow

act_flow=(A-B)/delta_t;
real_flow=[];
f_matrix=[];
for i=1:size(A,1)
    f_matrix=[f_matrix;-1/Z(i)    0           A(i,1)/Z(i)   A(i,1)*A(i,2)     -(1+A(i,1)^2)        A(i,2);
              0          -1/Z(i)     A(i,2)/Z(i)   (1+A(i,2)^2)      -A(i,1)*A(i,2)       -A(i,1)];
    real_flow=[real_flow;act_flow(i,1);act_flow(i,2)];
end
velocity=f_matrix\real_flow;

 if isempty(velocity)
     velocity=[0 0 0 0 0 0]';
 end

 beta=1;
for j=1:6
velocity(j)=beta*velocity(j)+(1-beta)*Velocity_old(j);
end

Velocity_old=velocity;
img_old=sensor.img;
delta_t_old= delta_t;
t=sensor.t;
VEL = R*[velocity(1) velocity(2) velocity(3) ]'+cross(velocity(4:6),Hcamtoimu(1:3,4));
OMG =R* [velocity(4) velocity(5) velocity(6) ]';
vel=[VEL(1) VEL(2) VEL(3)]';
omg=[OMG(1) OMG(2) OMG(3)]';


end
