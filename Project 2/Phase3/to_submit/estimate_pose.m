function [pos, q, camera_H] = estimate_pose(sensor,P0,P1,P2,P3,P4)
%ESTIMATE_POSE 6DOF pose estimator based on apriltags
%   sensor - struct stored in provided dataset, fields include
%          - is_ready: logical, indicates whether sensor data is valid
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
%              estimate_pose_handle = ...
%                  @(sensor) estimate_pose(sensor, your personal input arguments);
%   pos - 3x1 position of the quadrotor in world frame
%   q   - 4x1 quaternion of the quadrotor [w, x, y, z] where q = w + x*i + y*j + z*k

persistent p0w p1w p2w p3w p4w K;

if nargin>=5
    p0w=P0;
    p1w=P1;
    p2w=P2;
    p3w=P3;
    p4w=P4;
    K = [311.0520   0       201.8724;...
            0   311.3885    113.6210;...
            0       0             1];
end
    
%else        %implement is_ready
    if(isempty(sensor))
        pos=[];
        q=[];
        return;
    end
    
    M=[];
    for i=1:numel(sensor.id)
        tagid=sensor.id(i);
        
        m0=[-p0w(tagid+1,2) -p0w(tagid+1,3) -1 0 0 0 sensor.p0(1,i)*p0w(tagid+1,2) sensor.p0(1,i)*p0w(tagid+1,3) sensor.p0(1,i);...
            0 0 0 -p0w(tagid+1,2) -p0w(tagid+1,3) -1 sensor.p0(2,i)*p0w(tagid+1,2) sensor.p0(2,i)*p0w(tagid+1,3) sensor.p0(2,i)];
        
        m1=[-p1w(tagid+1,2) -p1w(tagid+1,3) -1 0 0 0 sensor.p1(1,i)*p1w(tagid+1,2) sensor.p1(1,i)*p1w(tagid+1,3) sensor.p1(1,i);...
            0 0 0 -p1w(tagid+1,2) -p1w(tagid+1,3) -1 sensor.p1(2,i)*p1w(tagid+1,2) sensor.p1(2,i)*p1w(tagid+1,3) sensor.p1(2,i)];
        
        m2=[-p2w(tagid+1,2) -p2w(tagid+1,3) -1 0 0 0 sensor.p2(1,i)*p2w(tagid+1,2) sensor.p2(1,i)*p2w(tagid+1,3) sensor.p2(1,i);...
            0 0 0 -p2w(tagid+1,2) -p2w(tagid+1,3) -1 sensor.p2(2,i)*p2w(tagid+1,2) sensor.p2(2,i)*p2w(tagid+1,3) sensor.p2(2,i)];
        
        m3=[-p3w(tagid+1,2) -p3w(tagid+1,3) -1 0 0 0 sensor.p3(1,i)*p3w(tagid+1,2) sensor.p3(1,i)*p3w(tagid+1,3) sensor.p3(1,i);...
            0 0 0 -p3w(tagid+1,2) -p3w(tagid+1,3) -1 sensor.p3(2,i)*p3w(tagid+1,2) sensor.p3(2,i)*p3w(tagid+1,3) sensor.p3(2,i)];
        
        m4=[-p4w(tagid+1,2) -p4w(tagid+1,3) -1 0 0 0 sensor.p4(1,i)*p4w(tagid+1,2) sensor.p4(1,i)*p4w(tagid+1,3) sensor.p4(1,i);...
            0 0 0 -p4w(tagid+1,2) -p4w(tagid+1,3) -1 sensor.p4(2,i)*p4w(tagid+1,2) sensor.p4(2,i)*p4w(tagid+1,3) sensor.p4(2,i)];
        
        M=[M;m0;m1;m2;m3;m4];
    end
    
    [U,D,V]=svd(M);
    
    last_col=V(:,end);
    H=[last_col(1:3)';last_col(4:6)';last_col(7:9)'];
    
    H=H/H(3,3); %NORMALIZING
    
    modH=K\H; %inv(K) is slower
    
    
    [modU,modS,modV]=svd([modH(:,1), modH(:,2), cross(modH(:,1),modH(:,2))]);
    
    R=modU*[1 0 0;0 1 0;0 0 det(modU*transpose(modV))]*transpose(modV);
    
    T=modH(:,3)/norm(modH(:,1));
    
    camera_H=[R,T];
    camera_H=[camera_H;0 0 0 1];
    
    rot=[1 0 0;0 cosd(180) -sind(180);0 sind(180) cosd(180)]*[cosd(45) -sind(45) 0;sind(45) cosd(45) 0;0 0 1];
    
    cameratoIMU_H=[rot,[-0.04; 0.0; -0.03]];
    cameratoIMU_H=[cameratoIMU_H;0 0 0 1];
    
    IMUtocamera_H=[inv(rot),-inv(rot)*[-0.04; 0.0; -0.03]];
    IMUtocamera_H=[IMUtocamera_H;0 0 0 1];
    
    IMU_H=IMUtocamera_H*camera_H;
    
    W_IMU_H=[inv(IMU_H(1:3,1:3)),-inv(IMU_H(1:3,1:3))*IMU_H(1:3,4)];
    W_IMU_H=[W_IMU_H;0 0 0 1];
    
    T=W_IMU_H(1:3,4);
    R=W_IMU_H(1:3,1:3);

    pos = T;
    q = RotToQuat(R);

%end

end