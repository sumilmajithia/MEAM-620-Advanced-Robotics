function [ desired_state ] = trajectory_generator(t, qn, map, path)
% TRAJECTORY_GENERATOR: Turn a Dijkstra or A* path into a trajectory
%
% NOTE: This function would be called with variable number of input
% arguments. In init_script, it will be called with arguments
% trajectory_generator([], [], map, path) and later, in test_trajectory,
% it will be called with only t and qn as arguments, so your code should
% be able to handle that. This can be done by checking the number of
% arguments to the function using the "nargin" variable, check the
% MATLAB documentation for more information.
%
% map: The map structure returned by your load_map function
% path: This is the path returned by your planner (dijkstra function)
%
% desired_state: Contains all the information that is passed to the
% controller, as in phase 2
%
% It is suggested to use "persistent" variables to store map and path
% during the initialization call of trajectory_generator, e.g.
persistent map0 path0 dummy X Y Z X_vel Y_vel Z_vel X_accel Y_accel Z_accel
if(isempty(t) && isempty(qn))
    map0 = map;
    path0 = path;
    dummy=1;
    waypoints=path0(1,1:3);
    for i= 3 : size(path0,1)
        if norm((path0(i,1:3)-path0(i-1,1:3))/norm(path0(i,1:3)-path0(i-1,1:3))-(path0(i-1,1:3)-path0(i-2,1:3))/norm(path0(i-1,1:3)-path0(i-2,1:3)))>0.05
            waypoints=[waypoints;path0(i,1:3)];
        end
    end
    s=1;
    j=2;

    newway=waypoints(1,1:3);
    while j~= size(waypoints,1)+1
        c=1;
    while c~=0
        if j>size(waypoints,1)
            break;
        end
        startpoint=waypoints(s,1:3);
        c=skipoints(map0,waypoints(j,1:3),startpoint);
        j=j+1;
    end
    s=j-2;
    newway=[newway;waypoints(j-2,1:3)];
    end
    v_av=3.20;
    dist = sqrt(diff(newway(:,1)).^2+diff(newway(:,2)).^2+diff(newway(:,3)).^2);
    T=sum(dist)/(v_av);
    T_sector=(dist/sum(dist))*T;
    T_qumulative=zeros((size(T_sector,1)+1),1);
    for k=1:size(T_sector)
        T_qumulative(k+1)=T_qumulative(k)+T_sector(k);
    end
    for k=1:size(T_sector)
        X(k,:)=quintic_trajectory(T_qumulative(k),T_qumulative(k+1),newway(k,1),newway(k+1,1),0,0);
        Y(k,:)=quintic_trajectory(T_qumulative(k),T_qumulative(k+1),newway(k,2),newway(k+1,2),0,0);
        Z(k,:)=quintic_trajectory(T_qumulative(k),T_qumulative(k+1),newway(k,3),newway(k+1,3),0,0);


        X_vel(k,:)=[X(k,2), 2*X(k,3), 3*X(k,4),4*X(k,5),5*X(k,6)];
        Y_vel(k,:)=[Y(k,2), 2*Y(k,3), 3*Y(k,4),4*Y(k,5),5*Y(k,6)];
        Z_vel(k,:)=[Z(k,2), 2*Z(k,3), 3*Z(k,4),4*Z(k,5),5*Z(k,6)];

        X_accel(k,:)=[2*X(3), 6*X(4) , 12*X(5), 20*X(6)];
        Y_accel(k,:)=[2*Y(3), 6*Y(4) , 12*Y(5), 20*Y(6)];
        Z_accel(k,:)=[2*Z(3), 6*Z(4) , 12*Z(5), 20*Z(6)];

    end
else
    disp(dummy);
end

time_x=[1, t, t^2, t^3, t^4, t^5];
tim_x_vel=[1, t, t^2, t^3, t^4];
tim_x_accel=[1, t, t^2, t^3];
for r=1:size(T_sector)+1
    if q>= T_qumulative(r) && q< T_qumulative(r+1)
        break;
    end
end
 
pos=[X(r,:)*time_x',Y(r,:)*time_x',Z(r,:)*time_x'];
vel=[X_vel(r,:)*tim_x_vel',Y_vel(r,:)*tim_x_vel',Z_vel(r,:)*tim_x_vel'];
acc=[X_accel(r,:)*tim_x_accel',Y_accel(r,:)*tim_x_accel',Z_accel(r,:)*tim_x_accel'];

yaw=0;
yawdot=0;


desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

desired_state=[];
end