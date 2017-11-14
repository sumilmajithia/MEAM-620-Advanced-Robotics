% Add additional inputs after the given ones if you want to
% Example:
% your_input = 1;
% ekf_handle1 = @(sensor, vic) eskf1(sensor, vic, your_input);
% ekf_handle2 = @(sensor) eskf2(sensor, your_input);
%
% We will only call ekf_handle in the test function.
% Note that this will only create a function handle, but not run the function
clc
clear all

load('studentdata9.mat')

p0=zeros(2,108);
p1=zeros(2,108);
p2=zeros(2,108);
p3=zeros(2,108);
p4=zeros(2,108);

empty.id=[];

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
vic.t=time;
vic.vel=[vicon(7,:);vicon(8,:);vicon(9,:);vicon(10,:);vicon(11,:);vicon(12,:)];
c=1;
for i=1:numel(time)
    V.t=vic.t(i);
    V.vel=vic.vel(:,i);   
    if time(i)==data(c).t      
        [X, Z]= ekf1(data(c),V,p0,p1,p2,p3,p4);
        c=c+1;
        if c> numel(data)
            c=c-1;
        end
    else
        [X, Z]= ekf1(data(c),V,p0,p1,p2,p3,p4);
    end
    if isempty(X)
        X=[NaN NaN NaN NaN NaN NaN NaN];
    end
    x(i)=X(1);
    y(i)=X(2);
    z(i)=X(3);
    q=[X(4) X(5) X(6) X(7)];
    R=QuatToRot(q);
    [roll(i), pitch(i), yaw(i)]= RotToRPY_ZXY(R);
   
end
subplot(3,2,1)
(plot(time,x));
hold on;plot(time,vicon(1,:));
subplot(3,2,3)
(plot(time,y));
hold on;plot(time,vicon(2,:));
subplot(3,2,5)
(plot(time,z));
hold on;plot(time,vicon(3,:));

subplot(3,2,2)
(plot(time,roll));
hold on;plot(time,vicon(4,:));
subplot(3,2,4)
(plot(time,pitch));
hold on;plot(time,vicon(5,:));
subplot(3,2,6)
(plot(time,yaw));
hold on;plot(time,vicon(6,:));

for i=1:numel(data)
    [A, B]= ekf2(data(i),p0,p1,p2,p3,p4);
    x1(i)=A(1);
    y1(i)=A(2);
    z1(i)=A(3);
    q1=[A(7) A(8) A(9) A(10)];
    R1=QuatToRot(q1);
    t(i)=data(i).t;
    [roll1(i), pitch1(i), yaw1(i)]= RotToRPY_ZXY((R1));
    vx(i)=A(4);
    vy(i)=A(5);
    vz(i)=A(6);
end
figure;
subplot(3,3,1)
(plot(t,x1));
hold on;plot(time,vicon(1,:));
subplot(3,3,4)
(plot(t,y1));
hold on;plot(time,vicon(2,:));
subplot(3,3,7)
(plot(t,z1));
hold on;plot(time,vicon(3,:));

subplot(3,3,2)
(plot(t,roll1));
hold on;plot(time,vicon(4,:));
subplot(3,3,5)
(plot(t,pitch1));
hold on;plot(time,vicon(5,:));
subplot(3,3,8)
(plot(t,yaw1));
hold on;plot(time,vicon(6,:));

subplot(3,3,3)
(plot(t,vx));
hold on;plot(time,vicon(7,:));
subplot(3,3,6)
(plot(t,vy));
hold on;plot(time,vicon(8,:));
subplot(3,3,9)
(plot(t,vz));
hold on;plot(time,vicon(9,:));