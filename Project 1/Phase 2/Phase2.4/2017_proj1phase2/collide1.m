function [C] = collide1(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
a=size(points);
C=zeros(a(1),1);
if (isempty(map))
    return;
end

for i=1:a(1)  
    for j=2:numel([map.Xmin])
%         if j==1
%             Xmin=map(j).Xmin;
%         Xmax=map(j).Xmax;
%         Ymin=map(j).Ymin;
%         Ymax=map(j).Ymax;
%         Zmin=map(j).Zmin;
%         Zmax=map(j).Zmax;
%         if points(i,1)<= Xmin&& points(i,1)>=Xmax && points(i,2)<= Ymin && points(i,2) >= Ymax && points(i,3)<=Zmin && points(i,3)>=Zmax
%            C(i)=1;
%            break;
%         end
%         else
        Xmin=map(j).Xmin;
        Xmax=map(j).Xmax;
        Ymin=map(j).Ymin;
        Ymax=map(j).Ymax;
        Zmin=map(j).Zmin;
        Zmax=map(j).Zmax;
        if points(i,1)>= Xmin&& points(i,1)<=Xmax && points(i,2)>= Ymin && points(i,2) <= Ymax && points(i,3)>=Zmin && points(i,3)<=Zmax
           C(i)=1;
           break;
        end
       
    end
    
end

end
