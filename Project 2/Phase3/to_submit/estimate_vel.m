function [vel, omg] = estimate_vel(sensor, varargin)
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
%              a data structure to represent the map or camera parameters,
%              your decision. But for the purpose of testing, since we don't
%              know what inputs you will use, you have to specify them in
%              init_script by doing
%              estimate_vel_handle = ...
%                  @(sensor) estimate_vel(sensor, your personal input arguments);
%   vel - 3x1 velocity of the quadrotor in world frame
%   omg - 3x1 angular velocity of the quadrotor
    persistent flag PointTracker old_points old_time old_img old_V old_del_t K
    
    if(isempty(sensor.id))
        vel=[];
        omg=[];
        return;
    end
    
    %% INITIALIZATION %%
    if(isempty(flag))
        first_corners=detectFASTFeatures(sensor.img);
        %first_corners=first_corners.selectStrongest(100);
        PointTracker = vision.PointTracker('MaxBidirectionalError',1);
        first_corners_location=first_corners.selectStrongest(500).Location;
        initialize(PointTracker,first_corners_location,sensor.img);
        
        flag=1;
        K=[311.0520   0       201.8724;...
              0   311.3885    113.6210;...
              0       0             1];
        old_points=first_corners_location;
        old_time=sensor.t;
        old_img=sensor.img;
        old_del_t=0;
        vel = zeros(3,1);
        omg = zeros(3,1);
        return;
    end
    
    %% UPDATING %%
    
    first_corners=detectFASTFeatures(old_img);
    %first_corners=first_corners.selectStrongest(100);
    PointTracker = vision.PointTracker('MaxBidirectionalError',1);
    first_corners_location=first_corners.selectStrongest(500).Location;
    initialize(PointTracker,first_corners_location,old_img);
    old_points=first_corners_location;
    [points, point_validity]=step(PointTracker,sensor.img);
%    setPoints(PointTracker,points,point_validity);
    
    dummy=K\[double(points) ones(size(points,1),1)]';
    camera_points=dummy(1:2,:)';
    
    dummy=K\[double(old_points) ones(size(old_points,1),1)]';
    old_camera_points=dummy(1:2,:)';
    
    
    [pos,q,camera_H]=estimate_pose(sensor);
    
    Z=-camera_H(3,4)./(camera_H(3,1)*camera_points(:,1)+camera_H(3,2)*camera_points(:,2)+camera_H(3,3));
   
    if flag==1
        alpha=1;
    else
        alpha=0.1;
    end
    flag=flag+1;
    del_t=alpha*(sensor.t-old_time)+(1-alpha)*old_del_t;
    opti_vel=(camera_points-old_camera_points)/del_t;%0.0205;%(sensor.t-old_time); 
    
    old_time=sensor.t;
    %old_points=points;
    old_img=sensor.img;
    old_del_t=del_t;
    
    %% NOT-RANSAC %%
    
    bigA=[];   bigB=[]; %finalA=[];  finalB=[];
    
    for j = 1:size(points,1) 
        x=camera_points(j,1);   y=camera_points(j,2);

        bigA=[bigA;-1/Z(j)    0       x/Z(j)  x*y     -(1+x^2)  y;
                     0    -1/Z(j)    y/Z(j)  (1+y^2) -x*y     -x];
        bigB=[bigB;opti_vel(j,1);opti_vel(j,2)];
    end
    
%    V=bigA\bigB;
    
    totiters=100;
    
    totpoints=zeros(totiters,10);
    
     for i=1:totiters
         
         inlier=0;
         %least_error=inf;
         %best_vel=[];
         
         random_points=randperm(size(opti_vel,1),3);
         %random_points=[100,200,300];
         A=[];
         B=[];
         
        for k=1:3
            j=random_points(k);
            x=camera_points(j,1);y=camera_points(j,2);
            
            A=[A;-1/Z(j)    0       x/Z(j)  x*y     -(1+x^2)  y;
                    0    -1/Z(j)    y/Z(j)  (1+y^2) -x*y     -x];
            B=[B;opti_vel(j,1);opti_vel(j,2);];
        end
             
         V=A\B;
         
         diff=bigB-bigA*V;
         
%          for i=1:2:numel(diff)-1
%              if rms(diff(i),diff(i+1))<0.01
%                  totpoints(
%                  
%              end
%          end
          
%         if(rms(diff)<0.01)
%             finalA=[finalA;A];
%             finalB=[finalB;B];
%         end
%  
     end
    %% main loop ends here
    
    %finalV=finalA\finalB;

%disp(size(A,1));
if(isempty(V))
    vel=zeros(3,1);
    omg=zeros(3,1);
else
    vel = camera_H(1:3,1:3)*V(1:3);
    omg = camera_H(1:3,1:3)*V(4:6);

end
end
