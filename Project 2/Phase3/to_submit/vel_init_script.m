% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function

%% ESTIMATE POSE INITIALIZATION %%
 p0=zeros(108,3);   p1=zeros(108,3);    p2=zeros(108,3);    
 p3=zeros(108,3);   p4=zeros(108,3);    x=0;    y=-2*0.152;
 
 for j = 1:9                   
     x=0; y=y+2*0.152;
     if j==4 || j==7
         y=y-0.152+0.178;
     end
     for i = 1:12
         
         p4((j-1)*12+i,:) = [(j-1)*12+i-1,x,y];
         p1((j-1)*12+i,:) = [(j-1)*12+i-1,x+0.152,y];
         p2((j-1)*12+i,:) = [(j-1)*12+i-1,x+0.152,y+0.152];
         p3((j-1)*12+i,:) = [(j-1)*12+i-1,x,y+0.152];
         p0((j-1)*12+i,:) = [(j-1)*12+i-1,x+0.076,y+0.076];
         
         x=x+2*0.152;
         
     end
 end

estimate_pose([],p0,p1,p2,p3,p4);
%estimate_pose_handle = @(sensor) estimate_pose(sensor,p0,p1,p2,p3,p4);

%% ESTIMATE VELOCITY INITIALIZATION %%
%estimate_vel_handle = @(sensor) estimate_vel(sensor);

[vel,omg] = estimate_vel(data(1));
