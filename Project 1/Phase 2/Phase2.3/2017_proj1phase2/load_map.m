function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
fileID=fopen(filename,'r');   % Create  a file ID to read data from
C = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f'); %reading the data
if(isempty([C{1}]))   % if empty return empty
    map=[];
    return;
end
Xmin=C{2};
Ymin=C{3};
Zmin=C{4};
Xmax=C{5};
Ymax=C{6};
Zmax=C{7};
map.xy_res=xy_res;
map.z_res=z_res;
l_x= (Xmax(1)-Xmin(1))/xy_res;
l_y=(Ymax(1)-Ymin(1))/xy_res;
l_z=(Zmax(1)-Zmin(1))/z_res;

Xmin(2:end)=Xmin(2:end)-margin;
Ymin(2:end)=Ymin(2:end)-margin;
Zmin(2:end)=Zmin(2:end)-margin;
Xmax(2:end)=Xmax(2:end)+margin;
Ymax(2:end)=Ymax(2:end)+margin;
Zmax(2:end)=Zmax(2:end)+margin;

% x_nodes=Xmin(1)+((1:((l_x+1)))-1)*xy_res;
% y_nodes=Ymin(1)+((1:((l_y+1)))-1)*xy_res;
% z_nodes=Zmin(1)+((1:((l_z+1)))-1)*z_res;
% 
% map((l_x+1)*(l_y+1)*(l_z+1)).x_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).y_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).z_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).cond=[];
% 
% MAP=combvec(x_nodes,y_nodes,z_nodes)';
% c=num2cell(MAP(:,1));[map(:).x_node]=deal(c{:});
% c=num2cell(MAP(:,2));[map(:).y_node]=deal(c{:});
% c=num2cell(MAP(:,3));[map(:).z_node]=deal(c{:});
% 
% for i=1:(l_x+1)*(l_y+1)*(l_z+1)
%     c=0;
%     for o=2:numel(Xmin)
%         if map(i).x_node>= Xmin(o)&& map(i).x_node<=Xmax(o) && map(i).y_node>= Ymin(o)&& map(i).y_node <= Ymax(o) && map(i).z_node>=Zmin(o) && map(i).z_node<=Zmax(o)
%             c=c+1;
%         end
%     end
%        
%     if c>0
%         map(i).cond=1;
%     else
%         map(i).cond=0;
%     end
% end                 
% end
x_nodes=Xmin(1)+((1:((l_x+1)))-1)*xy_res;
y_nodes=Ymin(1)+((1:((l_y+1)))-1)*xy_res;
z_nodes=Zmin(1)+((1:((l_z+1)))-1)*z_res;

% map((l_x+1)*(l_y+1)*(l_z+1)).x_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).y_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).z_node=[];
% map((l_x+1)*(l_y+1)*(l_z+1)).cond=[];

map=combvec(x_nodes,y_nodes,z_nodes)';
% c=num2cell(MAP(:,1));[map(:).x_node]=deal(c{:});
% c=num2cell(MAP(:,2));[map(:).y_node]=deal(c{:});
% c=num2cell(MAP(:,3));[map(:).z_node]=deal(c{:});

for i=1:(l_x+1)*(l_y+1)*(l_z+1)
    c=0;
    for o=2:numel(Xmin)
        if map(i,1)>= Xmin(o)&& map(i,1)<=Xmax(o) && map(i,1)>= Ymin(o)&& map(i,1) <= Ymax(o) && map(i,1)>=Zmin(o) && map(i,1)<=Zmax(o)
            c=c+1;
        end
    end
       
    if c>0
        map(4,i)=1;
    else
        map(4,i)=0;
    end
end                 
end
