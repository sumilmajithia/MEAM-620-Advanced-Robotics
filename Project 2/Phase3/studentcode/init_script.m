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
endp0=zeros(2,108);
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
end
    
%Assiging p0 cordinates
p0(1,i)=(p4(1,i)+p2(1,i))/2;
p0(2,i)=(p4(2,i)+p2(2,i))/2;
end
ekf1_handle = @(sensor, vic) ekf1(sensor, vic,p1,p1,p2,p3,p4);
ekf2_handle = @(sensor) ekf2(sensor,p0,p1,p2,p3,p4);
