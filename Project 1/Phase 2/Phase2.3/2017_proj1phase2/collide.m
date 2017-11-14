function [C] = collide(map, points)
% COLLIDE Test whether points collide with an obstacle in an environment.
%   C = collide(map, points).  points is an M-by-3 matrix where each
%   row is an (x, y, z) point.  C in an M-by-1 logical vector;
%   C(i) = 1 if M(i, :) touches an obstacle and is 0 otherwise.
a=size(points);
C=zeros(a(1),1);
if (isempty(map))
    return;
end
mapmodified=map([map(:).cond]==1);


for i=1:a(1)
    c=0;
    for j=1:numel(mapmodified)
        Xmin=mapmodified(j).x_node;
        Xmax=mapmodified(j).x_node+map(1).xy_res;
        Ymin=mapmodified(j).y_node;
        Ymax=mapmodified(j).y_node+map(1).xy_res;
        Zmin=mapmodified(j).z_node;
        Zmax=mapmodified(j).z_node+map(1).z_res;
        if points(i,1)>= Xmin&& points(i,1)<=Xmax && points(i,2)>= Ymin && points(i,2) <= Ymax && points(i,3)>=Zmin && points(i,3)<=Zmax
            c=c+1;
        end
    end
    if c>1
        C(i) = 1;
    else
        C(i)=0;
    end
end

end
