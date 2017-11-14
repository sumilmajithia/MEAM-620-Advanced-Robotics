clear all;
load('studentdata1.mat');
vel_init_script;
vic=struct('t',num2cell([time]),'vel',mat2cell([vicon(7:12,:)],[6],ones(1,1768)));

error_mat=[];

r=1;
%for r=0:0.02:1
%    disp(r);
    c=3;
    pos=[];

    ekf1(data(1),vic(1),r);T=[];

    for i=2:864
        for j=c:1768
            if vic(j).t==data(i).t
                [X, Z] = ekf1(data(i), vic(j),r);
                pos=[pos,X];
                c=j+1;
                break;
            else
                [X, Z] = ekf1([], vic(j),r);
                pos=[pos,X];
            end
        end
    end

    disp(norm(pos(1,:)-vicon(1,4:1768),2));
    disp(norm(pos(2,:)-vicon(2,4:1768),2));
    disp(norm(pos(3,:)-vicon(3,4:1768),2));

    subplot(3,2,1);plot(time(4:1768),pos(1,:),'r.',time,vicon(1,:),'b.');
    subplot(3,2,3);plot(time(4:1768),pos(2,:),'r.',time,vicon(2,:),'b.');
    subplot(3,2,5);plot(time(4:1768),pos(3,:),'r.',time,vicon(3,:),'b.');
    subplot(3,2,2);plot(time(4:1768),pos(4,:),'r.',time,vicon(4,:),'b.');
    subplot(3,2,4);plot(time(4:1768),pos(5,:),'r.',time,vicon(5,:),'b.');
    subplot(3,2,6);plot(time(4:1768),pos(6,:),'r.',time,vicon(6,:),'b.');
    
%end