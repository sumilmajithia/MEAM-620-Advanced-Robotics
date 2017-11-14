function plot_block(xmin, ymin, zmin, xmax, ymax, zmax,c)
fill3([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],[zmin,zmin,zmin,zmin],c);
fill3([xmin,xmax,xmax,xmin],[ymin,ymin,ymax,ymax],[zmax,zmax,zmax,zmax],c);
fill3([xmin,xmin,xmin,xmin],[ymin,ymin,ymax,ymax],[zmin,zmax,zmax,zmin],c);
fill3([xmax,xmax,xmax,xmax],[ymin,ymin,ymax,ymax],[zmin,zmax,zmax,zmin],c);
fill3([xmin,xmin,xmax,xmax],[ymin,ymin,ymin,ymin],[zmin,zmax,zmax,zmin],c);
fill3([xmin,xmin,xmax,xmax],[ymax,ymax,ymax,ymax],[zmin,zmax,zmax,zmin],c);
end