function A=neighbours(point,xy_res,z_res)
A=[point(1)+xy_res, point(2),        point(3);
   point(1)-xy_res, point(2),        point(3);
   point(1),        point(2)+xy_res, point(3);
   point(1),        point(2)-xy_res, point(3);
   point(1),        point(2),        point(3)+z_res;
   point(1),        point(2),        point(3)-z_res];
end