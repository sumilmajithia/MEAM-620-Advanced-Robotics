function [path, num_expanded] = dijkstra(map, start, goal, astar)
% DIJKSTRA Find the shortest path from start to goal.
%   PATH = DIJKSTRA(map, start, goal) returns an M-by-3 matrix, where each row
%   consists of the (x, y, z) coordinates of a point on the path.  The first
%   row is start and the last row is goal.  If no path is found, PATH is a
%   0-by-3 matrix.  Consecutive points in PATH should not be farther apart than
%   neighboring cells in the map (e.g.., if 5 consecutive points in PATH are
%   co-linear, don't simplify PATH by removing the 3 intermediate points).
%
%   PATH = DIJKSTRA(map, start, goal, astar) finds the path using euclidean
%   distance to goal as a heuristic if astar is true.
%
%   [PATH, NUM_EXPANDED] = DIJKSTRA(...) returns the path as well as
%   the number of points that were visited while performing the search.
if (isempty(map))
   path= double.empty(0,3);
    return;
end
if nargin < 4
    astar = false;
end
path=[];

   
    startnode(1)= round((start(1)-map(1).Xmin)/map(1).xy_res,0)*map(1).xy_res+map(1).Xmin;
    startnode(2)= round((start(2)-map(1).Ymin)/map(1).xy_res,0)*map(1).xy_res+map(1).Ymin;
    startnode(3)= round((start(3)-map(1).Zmin)/map(1).z_res,0)*map(1).z_res+map(1).Zmin;
    
    goalnode(1)= round((goal(1)-map(1).Xmin)/map(1).xy_res,0)*map(1).xy_res+map(1).Xmin;
    goalnode(2)= round((goal(2)-map(1).Ymin)/map(1).xy_res,0)*map(1).xy_res+map(1).Ymin;
    goalnode(3)= round((goal(3)-map(1).Zmin)/map(1).z_res,0)*map(1).z_res+map(1).Zmin;
    
    x_nodes=map(1).Xmin+((1:((map(1).l_x)))-1)*map(1).xy_res;
    y_nodes=map(1).Ymin+((1:((map(1).l_y)))-1)*map(1).xy_res;
    z_nodes=map(1).Zmin+((1:((map(1).l_z)))-1)*map(1).z_res;
    
    Q=combvec(x_nodes,y_nodes,z_nodes)';
    Q=round(Q,4);
    startnodeindex=find(ismember(Q,startnode,'rows'));
    goalnodeindex=find(ismember(Q,goalnode,'rows'));
    g=inf(size(Q,1),1) ;
    map(1).l_x=map(1).l_x-mod(map(1).l_x,1);
    map(1).l_y=map(1).l_y-mod(map(1).l_y,1);
    map(1).l_z=map(1).l_z-mod(map(1).l_z,1);
   
    
nodeindexarray=1:map(1).l_x*map(1).l_y*map(1).l_z;
P=zeros(map(1).l_x*map(1).l_y*map(1).l_z,1) ; % visited node
Q_status=ones(map(1).l_x*map(1).l_y*map(1).l_z,1);
num_expanded = 0;
indexarray=1:map(1).l_x*map(1).l_y*map(1).l_z;
[g,indexarray,nodeindexarray]=updateheap(g,indexarray,0,startnodeindex,nodeindexarray);
 node_condition= collide(map,Q); 

while (Q_status(goalnodeindex)) && ((g(1))<inf)
    [g,minvalue,index,indexarray,nodeindexarray]=extractmin(g,indexarray,nodeindexarray);   
    Q_status(index)=0;
    %point_3d=Q(index,1:3);
    C=neighbours(index,map(1).l_x,map(1).l_y,map(1).l_z);
   % neighbourpoints=round(neighbourpoints,4);
    %C=find(ismember(Q,neighbourpoints,'rows'));
     C=C((Q_status(C)==1));
     C=C((node_condition(C)==0));
   %C=C((collide(map,Q(C,1:3))==0));
    for s=1:numel(C)
        d=minvalue+sqrt((Q(C(s),1)-Q(index,1))^2+(Q(C(s),2)-Q(index,2))^2+(Q(C(s),3)-Q(index,3))^2);%sum(abs(Q(C(s),1:3)-Q(index,1:3)));       
        if astar
        d=d+sqrt((Q(C(s),1)-Q(goalnodeindex,1))^2+(Q(C(s),2)-Q(goalnodeindex,2))^2+(Q(C(s),3)-Q(goalnodeindex,3))^2)-minvalue;%pdist([Q(C(s),1) Q(C(s),2) Q(C(s),3);Q(goalnodeindex,1) Q(goalnodeindex,2) Q(goalnodeindex,3)]);
        end
        if d<g(nodeindexarray(C(s)))
            [g,indexarray,nodeindexarray]=updateheap(g,indexarray,d,nodeindexarray(C(s)),nodeindexarray);
            P(C(s))=index;
        end
    end
    num_expanded = num_expanded +1;
%   
end
path=[goalnode;goal];
parentindex=P(goalnodeindex);
while parentindex ~= 0
    path=[Q(parentindex,1:3);path];
    parentindex=P(parentindex);
end

path=[start;path];
end

