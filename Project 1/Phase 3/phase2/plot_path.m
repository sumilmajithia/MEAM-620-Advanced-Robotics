function plot_path(map, path)
plot3(path(:,1),path(:,2),path(:,3));
hold on 
 for i=2:numel([map.Xmin])
     plot_block(map(i).Xmin, map(i).Ymin, map(i).Zmin, map(i).Xmax, map(i).Ymax, map(i).Zmax,[1 0 0])
     hold on
end