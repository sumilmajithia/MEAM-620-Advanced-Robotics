% add additional inputs after sensor if you want to
% Example:
% your_input = 1;
% estimate_pose_handle = @(sensor) estimate_pose(sensor, your_input);
% We will only call estimate_pose_handle in the test function.
% Note that unlike project 1 phase 3, thise will only create a function
% handle, but not run the function at all.

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
% estimate_pose([],p0,p1,p2,p3,p4);


estimate_pose_handle = @(sensor) estimate_pose(sensor,p0,p1,p2,p3,p4);

 for i =1:numel(data)
    [T,q]= estimate_pose(data(i),p0,p1,p2,p3,p4);
    Xpos(i)=T(1);
    Ypos(i)=T(2);
    Zpos(i)=T(3);
    t(i)=data(i).t;
    R=QuatToRot(q);
    [r(i) P(i) Y(i)]=RotToRPY_ZXY(R');
    
end
subplot(3,2,1)
(plot(t,Xpos));
hold on;plot(time,vicon(1,:));
subplot(3,2,3)
(plot(t,Ypos));
hold on;plot(time,vicon(2,:));
subplot(3,2,5)
(plot(t,Zpos));
hold on;plot(time,vicon(3,:));


subplot(3,2,2)
(plot(t,r));
hold on;plot(time,vicon(4,:));
subplot(3,2,4)
(plot(t,P));
hold on;plot(time,vicon(5,:));
subplot(3,2,6)
(plot(t,Y));
hold on;plot(time,vicon(6,:));