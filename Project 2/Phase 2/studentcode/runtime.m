% Add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_vel_handle = @(sensor) estimate_vel(sensor, your_input);
%
% We will only call estimate_vel_handle in the test function.
% Note that thise will only create a function handle, but not run the function
clc
clear all
load('E:\Advanced robotics\Project 2\Phase 2\studentcode\data\studentdata1.mat')

% estimate_vel_handle = @(sensor) estimate_vel(sensor);
p0=zeros(2,108);
p1=zeros(2,108);
p2=zeros(2,108);
p3=zeros(2,108);
p4=zeros(2,108);

for i= 1:108
    
% Assigning P1 cordinate 
    if mod(i,12)>0
        p1(1,i)= 0.152*(2*mod(i,12)-1);
    else 
        p1(1,i)=0.152*(2*12-1);
    end
    if ceil(i/12)<4
        if mod(i,12)==0
        p1(2,i)= 0.152*(2*(floor(i/12)-1));
        else
            p1(2,i)= 0.152*(2*(floor(i/12)));
        end
    else if ceil(i/12)>=4 && ceil(i/12)<7
             if mod(i,12)==0
        p1(2,i)= 0.152*(2*(floor(i/12)-1))+0.026;
        else
            p1(2,i)= 0.152*(2*(floor(i/12)))+0.026;
             end
        else
            if mod(i,12)==0
        p1(2,i)= 0.152*(2*(floor(i/12)-1))+0.052;
        else
            p1(2,i)= 0.152*(2*(floor(i/12)))+0.052;
            end
        end
    end
% Assigning P2 cordinate
    if mod(i,12)>0
        p2(1,i)= 0.152*(2*mod(i,12)-1);
    else 
        p2(1,i)=0.152*(2*12-1);
    end
    if ceil(i/12)<4
        if mod(i,12)==0
        p2(2,i)= 0.152*(2*(floor(i/12)-1))+0.152;
        else
            p2(2,i)= 0.152*(2*(floor(i/12)))++0.152;
        end
    else if ceil(i/12)>=4 && ceil(i/12)<7
             if mod(i,12)==0
        p2(2,i)= 0.152*(2*(floor(i/12)-1))+0.026++0.152;
        else
            p2(2,i)= 0.152*(2*(floor(i/12)))+0.026++0.152;
             end
        else
            if mod(i,12)==0
        p2(2,i)= 0.152*(2*(floor(i/12)-1))+0.052+0.152;
        else
            p2(2,i)= 0.152*(2*(floor(i/12)))+0.052++0.152;
            end
        end
    end
% Assigning P3 cordinate 
    if mod(i,12)>0
        p3(1,i)= 0.152*(2*mod(i,12)-1)-0.152;
    else 
        p3(1,i)=0.152*(2*12-1)-0.152;
    end
    if ceil(i/12)<4
        if mod(i,12)==0
        p3(2,i)= 0.152*(2*(floor(i/12)-1))+0.152;
        else
            p3(2,i)= 0.152*(2*(floor(i/12)))+0.152;
        end
    else if ceil(i/12)>=4 && ceil(i/12)<7
             if mod(i,12)==0
        p3(2,i)= 0.152*(2*(floor(i/12)-1))+0.026+0.152;
        else
            p3(2,i)= 0.152*(2*(floor(i/12)))+0.026+0.152;
             end
        else
            if mod(i,12)==0
        p3(2,i)= 0.152*(2*(floor(i/12)-1))+0.052+0.152;
        else
            p3(2,i)= 0.152*(2*(floor(i/12)))+0.052+0.152;
            end
        end
    end
    
% Assigning points for p4
    if mod(i,12)>0
        p4(1,i)= 0.152*(2*mod(i,12)-1)-0.152;
    else 
        p4(1,i)=0.152*(2*12-1)-0.152;
    end
    if ceil(i/12)<4
        if mod(i,12)==0
        p4(2,i)= 0.152*(2*(floor(i/12)-1));
        else
            p4(2,i)= 0.152*(2*(floor(i/12)));
        end
    else if ceil(i/12)>=4 && ceil(i/12)<7
             if mod(i,12)==0
        p4(2,i)= 0.152*(2*(floor(i/12)-1))+0.026;
             else
            p4(2,i)= 0.152*(2*(floor(i/12)))+0.026;
             end
        else
            if mod(i,12)==0
        p4(2,i)= 0.152*(2*(floor(i/12)-1))+0.052;
        else
            p4(2,i)= 0.152*(2*(floor(i/12)))+0.052;
            end
        end
    end
    
%Assiging p0 cordinates
p0(1,i)=(p4(1,i)+p2(1,i))/2;
p0(2,i)=(p4(2,i)+p2(2,i))/2;
end
for i=1:numel(data)

[V, O] = estimate_vel(data(i),p0,p1,p2,p3,p4);
if isempty(V)
V_x(i)=0;
V_y(i)=0;
V_z(i)=0;
O_x(i)=0;
O_y(i)=0;
O_z(i)=0;
t(i)=data(i).t;
else  
  i
V_x(i)=V(1);
V_y(i)=V(2);
V_z(i)=V(3);
O_x(i)=O(1);
O_y(i)=O(2);
O_z(i)=O(3);
t(i)=data(i).t;
end
end
subplot(3,2,1)
(plot(t,V_x));
hold on;plot(time,vicon(7,:));
ylim([-2 2])
subplot(3,2,3)
(plot(t,V_y));
hold on;plot(time,vicon(8,:));
ylim([-2 2])
subplot(3,2,5)
(plot(t,V_z));
hold on;plot(time,vicon(9,:));
ylim([-2 2])


subplot(3,2,2)
(plot(t,O_x));
hold on;plot(time,vicon(10,:));
ylim([-2 2])
subplot(3,2,4)
(plot(t,O_y));
hold on;plot(time,vicon(11,:));
ylim([-2 2])
subplot(3,2,6)
(plot(t,O_z));
hold on;plot(time,vicon(12,:));
ylim([-2 2])

