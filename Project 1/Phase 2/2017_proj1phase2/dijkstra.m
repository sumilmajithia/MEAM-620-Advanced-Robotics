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
% path, num_expanded
%% Check in which element the start part belongs. If outside the boundry retun no path  %%
I=1;
j=1;
while(I)
    if j>numel(map)
        path=zeros(0,3);
        break;
    end
        Xmin=map(j).x_node;
        Xmax=map(j).x_node+map(1).xy_res;
        Ymin=map(j).y_node;
        Ymax=map(j).y_node+map(1).xy_res;
        Zmin=map(j).z_node;
        Zmax=map(j).z_node+map(1).z_res;
        if start(1)>= Xmin&& start(1)<=Xmax && start(2)>= Ymin && start(2) <= Ymax && start(3)>=Zmin && start(3)<=Zmax
            I=0;
        end
        j=j+1;
end
j=j-1;  % element in which the start point belong
%% Check in which element the goal part belongs. If outside the boundry retun no path  %%
O=1;
i=1;
while(O)
    if i>numel(map)
         path=zeros(0,3);
        break;
    end
        Xmin=map(i).x_node;
        Xmax=map(i).x_node+map(1).xy_res;
        Ymin=map(i).y_node;
        Ymax=map(i).y_node+map(1).xy_res;
        Zmin=map(i).z_node;
        Zmax=map(i).z_node+map(1).z_res;
        if goal(1)>= Xmin&& goal(1)<=Xmax && goal(2)>= Ymin && goal(2) <= Ymax && goal(3)>=Zmin && goal(3)<=Zmax
           O=0;
        end
        i=i+1;
end
i=i-1;  % element in which the end point belong

%% Check if either the start or end is inside the obstacle. If outside the boundry retun no path  %%
k=1;
 mapmodified=map([map(:).cond]==1);
 
while (1)
    if k>numel(mapmodified)
        break;
    end
    Xmin=mapmodified(k).x_node;
    Xmax=mapmodified(k).x_node+map(1).xy_res;
    Ymin=mapmodified(k).y_node;
    Ymax=mapmodified(k).y_node+map(1).xy_res;
    Zmin=mapmodified(k).z_node;
    Zmax=mapmodified(k).z_node+map(1).z_res;
        if (start(1)>= Xmin&& start(1)<=Xmax && start(2)>= Ymin && start(2) <= Ymax && start(3)>=Zmin && start(3)<=Zmax)...
                ||(goal(1)>= Xmin&& goal(1)<=Xmax && goal(2)>= Ymin && goal(2) <= Ymax && goal(3)>=Zmin && goal(3)<=Zmax)
            path=zeros(0,3);
            
            break;
        end
        k=k+1;
end
k=k-1; 

%% Create the nodal space from elemental space.
First_node=[map(1).x_node,map(1).y_node,map(1).z_node];
Last_node=[map(numel(map)).x_node+map(1).xy_res,map(numel(map)).y_node+map(1).xy_res,map(numel(map)).z_node+map(1).z_res];
node=[];

for z=(First_node(3)):map(1).z_res:Last_node(3)
    for y=First_node(2):map(1).xy_res:Last_node(2)
        for x=First_node(1):map(1).xy_res:Last_node(1)
            node=[node;x,y,z];
            
        end
    end
end
  %node_condition= collide(map,node);   
  node_condition= map(:).cond; 
%% Finding the starting and ending nodes

Elstart=eltono(map(j).x_node,map(j).y_node,map(j).z_node,map(1).xy_res,map(1).z_res);
for u=1:8
    mindisstart(u)=sqrt((start(1)-Elstart(u,1))^2+(start(2)-Elstart(u,2))^2+(start(3)-Elstart(u,3))^2);
end
[~, indexstart]=min(mindisstart);
start_node=[Elstart(indexstart,1),Elstart(indexstart,2),Elstart(indexstart,3)];
startnodeindex=find(ismember(node,start_node,'rows'),1);

Elgoal=eltono(map(i).x_node,map(i).y_node,map(i).z_node,map(1).xy_res,map(1).z_res);
for u=1:8
    mindisgoal(u)=sqrt((goal(1)-Elgoal(u,1))^2+(goal(2)-Elgoal(u,2))^2+(goal(3)-Elgoal(u,3))^2);
end
  
[~, indexgoal]=min(mindisgoal);

goal_node=[Elgoal(indexgoal,1),Elgoal(indexgoal,2),Elgoal(indexgoal,3)];
goalnodeindex=find(ismember(node,goal_node,'rows'),1);
%%
g=inf(size(node,1),1) ;
g(startnodeindex)=0;
Q=node;
P=zeros(size(node,1),1) ; % visited node
Q_status=ones(size(node,1),1);
num_expanded = 0;
%(ismember(Q,goal_node,'rows'))
while (Q_status(goalnodeindex)) && (min(g(Q_status==1))<inf)
    [~, index]= min(g(Q_status==1));
    Q_status(index)=0;
    point_3d=Q(index,1:3);
    neighbourpoints=neighbours(point_3d,map(1).xy_res,map(1).z_res);
    C=find(ismember(Q,neighbourpoints,'rows'));
    C=C((Q_status(C)==1));
    C=C((node_condition(C)==0));
    for i=1:numel(C)
        d=g(index)+sum(abs(Q(C(i),1:3)-Q(index,1:3)));
        if d<g(C(i))
            g(C(i))=d;
            P(C(i))=index;
        end
    end
    num_expanded = num_expanded +1;
end

if nargin < 4
    astar = false;
end

path=[goal_node;goal];
parentindex=P(goalnodeindex);
while parentindex ~= 0
    path=[Q(parentindex);path];
    parentindex=P(parentindex);
end

path=[start;path];
end
