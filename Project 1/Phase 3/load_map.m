function map = load_map(filename, xy_res, z_res, margin)
% LOAD_MAP Load a map from disk.
%  MAP = LOAD_MAP(filename, xy_res, z_res, margin).  Creates an occupancy grid
%  map where a node is considered fill if it lies within 'margin' distance of
%  on abstacle.
fileID=fopen(filename,'r');   % Create  a file ID to read data from
 C = textscan(fileID,'%s %f %f %f %f %f %f %f %f %f ','commentStyle','#');
 
entity=C{1};
bindex=find(ismember(entity,'boundary'));



xxmin=C{2};
yymin=C{3};
zzmin=C{4};
xxmax=C{5};
yymax=C{6};
zzmax=C{7};

Xmin(1)=xxmin(bindex);
xxmin(bindex)=[];
Ymin(1)=yymin(bindex);
yymin(bindex)=[];
Zmin(1)=zzmin(bindex);
zzmin(bindex)=[];
Xmax(1)=xxmax(bindex);
xxmax(bindex)=[];
Ymax(1)=yymax(bindex);
yymax(bindex)=[];
Zmax(1)=zzmax(bindex);
zzmax(bindex)=[];
Xmin=[Xmin;xxmin(:)];
Xmax=[Xmax;xxmax(:)];
Ymin=[Ymin;yymin(:)];
Ymax=[Ymax;yymax(:)];
Zmin=[Zmin;zzmin(:)];
Zmax=[Zmax;zzmax(:)];
% Xmin=C{2};
% Ymin=C{3};
% Zmin=C{4};
% Xmax=C{5};
% Ymax=C{6};
% Zmax=C{7};
map.xy_res=xy_res;
map.z_res=z_res;
map.l_x= ((Xmax(1)-Xmin(1))/xy_res)+1;
map.l_y=((Ymax(1)-Ymin(1))/xy_res)+1;
map.l_z=((Zmax(1)-Zmin(1))/z_res)+1;

% map.l_x=map.l_x-mod(map.l_x,1); 
% map.l_y=map.l_y-mod(map.l_y,1);
% map.l_z=map.l_z-mod(map.l_z,1);

if map.l_x<=0
    map.l_x=1;
end
if map.l_y<=0
    map.l_y=1;
end
if map.l_z<=0
    map.l_z=1;
end


Xmin(2:end)=Xmin(2:end)-margin;
Xmax(2:end)=Xmax(2:end)+margin;
Ymin(2:end)=Ymin(2:end)-margin;
Ymax(2:end)=Ymax(2:end)+margin;
Zmin(2:end)=Zmin(2:end)-margin;
Zmax(2:end)=Zmax(2:end)+margin;

for i=1:(numel(Xmin))

map(i).Xmin=Xmin(i);
map(i).Xmax=Xmax(i);
map(i).Ymin=Ymin(i);
map(i).Ymax=Ymax(i);
map(i).Zmin=Zmin(i);
map(i).Zmax=Zmax(i);
end


end

