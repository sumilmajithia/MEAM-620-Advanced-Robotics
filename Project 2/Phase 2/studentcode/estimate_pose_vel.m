function [T, R ,Hcamtoimu] = estimate_pose(sensor,p0,p1,p2,p3,p4)
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

% persistent p0 p1 p2 p3 p4
% if isempty(sensor)
% p0=P0;
% p1=P1;
% p2=P2;
% p3=P3;
% p4=P4;
% return;
% end

M1=zeros(0,9);
if isempty(sensor.id)
    pos=[];
    q=[];
    return;
end
% if sensor.is_ready
% id=sensor.id+1;
% sensorp01=sensor.p0(1,:);
% sensorp02=sensor.p0(2,:);
% sensorp11=sensor.p1(1,:);
% sensorp12=sensor.p1(2,:);
% sensorp21=sensor.p2(1,:);
% sensorp22=sensor.p2(2,:);
% sensorp31=sensor.p3(1,:);
% sensorp32=sensor.p3(2,:);
% sensorp41=sensor.p4(1,:);
% sensorp42=sensor.p4(2,:);
% i=1:numel(sensor.id);
% M1=[M1;
%     -p0(1,id(i)) -p0(2,id(i)) -1 0 0 0 p0(1,id(i)).*sensorp01 p0(2,id(i)).*sensorp01 sensorp01;
%     0 0 0 -p0(1,id) -p0(2,id) -1 p0(1,id).*sensorp02 p0(2,id).*sensorp02 sensorp02;
%     -p1(1,id) -p1(2,id) -1 0 0 0 p1(1,id).*sensorp11 p1(2,id).*sensorp11 sensorp11;
%     0 0 0 -p1(1,id) -p1(2,id) -1 p1(1,id).*sensorp12 p0(2,id).*sensorp12 sensorp12;
%     -p2(1,id) -p2(2,id) -1 0 0 0 p2(1,id).*sensorp21 p2(2,id).*sensorp21 sensorp21;
%     0 0 0 -p2(1,id) -p2(2,id) -1 p2(1,id).*sensorp22 p2(2,id).*sensorp22 sensorp22;
%     -p3(1,id) -p3(2,id) -1 0 0 0 p3(1,id).*sensorp31 p3(2,id).*sensorp31 sensorp31;
%     0 0 0 -p3(1,id) -p3(2,id) -1 p3(1,id).*sensorp02 p3(2,id).*sensorp32 sensorp32;
%     -p4(1,id) -p4(2,id) -1 0 0 0 p4(1,id).*sensorp41 p4(2,id).*sensorp41 sensorp41;
%     0 0 0 -p4(1,id) -p4(2,id) -1 p4(1,id).*sensorp42 p4(2,id).*sensorp42 sensorp42];
% end
if sensor.is_ready
    for i=1:numel(sensor.id)       
            M1=[M1;-p0(1,sensor.id(i)+1) -p0(2,sensor.id(i)+1) -1 0 0 0 p0(1,sensor.id(i)+1)*sensor.p0(1,i)...
                p0(2,sensor.id(i)+1)*sensor.p0(1,(i)) sensor.p0(1,(i))];
            M1=[M1;0 0 0 -p0(1,sensor.id(i)+1) -p0(2,sensor.id(i)+1) -1 p0(1,sensor.id(i)+1)*sensor.p0(2,i)...
                p0(2,sensor.id(i)+1)*sensor.p0(2,(i)) sensor.p0(2,(i))];
            
             M1=[M1;-p1(1,sensor.id(i)+1) -p1(2,sensor.id(i)+1) -1 0 0 0 p1(1,sensor.id(i)+1)*sensor.p1(1,i)...
                p1(2,sensor.id(i)+1)*sensor.p1(1,(i)) sensor.p1(1,(i))];
             M1=[M1;0 0 0 -p1(1,sensor.id(i)+1) -p1(2,sensor.id(i)+1) -1 p1(1,sensor.id(i)+1)*sensor.p1(2,i)...
                p1(2,sensor.id(i)+1)*sensor.p1(2,(i)) sensor.p1(2,(i))];
            
             M1=[M1;-p2(1,sensor.id(i)+1) -p2(2,sensor.id(i)+1) -1 0 0 0 p2(1,sensor.id(i)+1)*sensor.p2(1,i)...
                p2(2,sensor.id(i)+1)*sensor.p2(1,(i)) sensor.p2(1,(i))];
             M1=[M1;0 0 0 -p2(1,sensor.id(i)+1) -p2(2,sensor.id(i)+1) -1 p2(1,sensor.id(i)+1)*sensor.p2(2,i)...
                p2(2,sensor.id(i)+1)*sensor.p2(2,(i)) sensor.p2(2,(i))];
            
             M1=[M1;-p3(1,sensor.id(i)+1) -p3(2,sensor.id(i)+1) -1 0 0 0 p3(1,sensor.id(i)+1)*sensor.p3(1,i)...
                p3(2,sensor.id(i)+1)*sensor.p3(1,(i)) sensor.p3(1,(i))];
             M1=[M1;0 0 0 -p3(1,sensor.id(i)+1) -p3(2,sensor.id(i)+1) -1 p3(1,sensor.id(i)+1)*sensor.p3(2,i)...
                p3(2,sensor.id(i)+1)*sensor.p3(2,(i)) sensor.p3(2,(i))];
            
             M1=[M1;-p4(1,sensor.id(i)+1) -p4(2,sensor.id(i)+1) -1 0 0 0 p4(1,sensor.id(i)+1)*sensor.p4(1,i)...
                p4(2,sensor.id(i)+1)*sensor.p4(1,(i)) sensor.p4(1,(i))];     
             M1=[M1;0 0 0 -p4(1,sensor.id(i)+1) -p4(2,sensor.id(i)+1) -1 p4(1,sensor.id(i)+1)*sensor.p4(2,i)...
                p4(2,sensor.id(i)+1)*sensor.p4(2,(i)) sensor.p4(2,(i))];


    end
end

[~, ~, V]=svd(M1);
lc=V(1:9,9);

    lc=lc./lc(9);

H=[lc(1) lc(2) lc(3);
    lc(4) lc(5) lc(6);
    lc(7) lc(8) lc(9)];
% K=[314.1779 0         199.4848; ...
%    0         314.2218  113.7838; ...
%    0         0         1];
K = [311.0520 0 201.8724; 0 311.3885 113.6210; 0 0 1];
H=K\H;
h1=H(1:3,1);
h2=H(1:3,2);
h3=H(1:3,3);
mod_H=[h1,h2,cross(h1,h2)];

% [u, ~, v]=svd(mod_H);
% Rcamtoimu=[cos(pi/4) -cos(pi/4) 0;
%            -sin(pi/4) -sin(pi/4) 0;
%            0 0 -1];
% R=Rcamtoimu*u*[1 0 0;0 1 0; 0 0 det((u*transpose(v)))]*transpose(v);
% c
% Hcamtoimu=[Rcamtoimu(1,1:3),0.04;Rcamtoimu(2,1:3) 0;Rcamtoimu(3,1:3),0.03;0 0 0 1];
% Rcamtoworld=u*[1 0 0;0 1 0; 0 0 det((u*transpose(v)))]*transpose(v);
% Hcamtoworld=[Rcamtoworld(1,1:3),T(1);Rcamtoworld(2,1:3),T(2);Rcamtoworld(3,1:3),T(3);0,0,0,1];
% 
% new_h=(Hcamtoimu*Hcamtoworld);
% r_inverse=inv([new_h(1,1:3);new_h(2,1:3);new_h(3,1:3)]);
% T=-r_inverse*new_h((1:3),4);
%        
[u, ~, v]=svd(mod_H);
R=u*[1 0 0;0 1 0; 0 0 det((u*transpose(v)))]*transpose(v);
T= (h3/norm(h1));
Hcamtoworld=[R,T;0,0,0,1];

Rcamtoimu=[cos(pi/4) -cos(pi/4) 0;
           -sin(pi/4) -sin(pi/4) 0;
           0 0 -1];
  Tci=[-0.04,0.0,-0.03];
  Hcamtoimu=[inv(Rcamtoimu),-inv(Rcamtoimu)*Tci';0, 0, 0, 1];
%    new_h=(Hcamtoimu*Hcamtoworld);
%    
%    r_inverse=inv([new_h(1,1:3);new_h(2,1:3);new_h(3,1:3)]);
% T=-r_inverse*new_h((1:3),4);
% 
% pos = T;
% q=r_inverse;





%     error=0;
%     
%     for i=1:numel(sensor.id)
%         tagid=sensor.id(i);
%         
%         dummy=H*[p0(1,tagid+1);p0(2,tagid+1);1];
%         u=dummy(1)/dummy(3);  v=dummy(2)/dummy(3);
%         error=error+(u-sensor.p0(1,i))^2+(v-sensor.p0(2,i))^2;
%         
%         dummy=H*[p1(1,tagid+1);p1(2,tagid+1);1];
%         u=dummy(1)/dummy(3);  v=dummy(2)/dummy(3);
%         error=error+(u-sensor.p1(1,i))^2+(v-sensor.p1(2,i))^2;
%         
%         dummy=H*[p2(1,tagid+1);p2(2,tagid+1);1];
%         u=dummy(1)/dummy(3);  v=dummy(2)/dummy(3);
%         error=error+(u-sensor.p2(1,i))^2+(v-sensor.p2(2,i))^2;
%         
%         dummy=H*[p3(1,tagid+1);p3(2,tagid+1);1];
%          u=dummy(1)/dummy(3);  v=dummy(2)/dummy(3);
%         error=error+(u-sensor.p3(1,i))^2+(v-sensor.p3(2,i))^2;
%         
%         dummy=H*[p4(1,tagid+1);p4(2,tagid+1);1];
%          u=dummy(1)/dummy(3);  v=dummy(2)/dummy(3);
%         error=error+(u-sensor.p4(1,i))^2+(v-sensor.p4(2,i))^2;
%         
%     end
%     
%     disp(sqrt(error/(2*5*numel(sensor.id))));



end
