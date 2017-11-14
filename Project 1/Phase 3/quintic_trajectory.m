function X = quintic_trajectory(tstart, tend, qstart, qend, qdotstart, qdotend)
% Quintic interpolation.

% By default, do linear interpolation.  You must replace this with the
% correct trajectory type.  Be sure to use the qdotstart and qdotend
% arguments (starting and ending velocities).  Use zero for the
% starting and ending accelerations.
A=[1,tstart,tstart^2,tstart^3, tstart^4,tstart^5;
    0,1,2*tstart,3*tstart^2,4*tstart^3,5*tstart^4;
    0,0,2,6*tstart,12*tstart^2,20*tstart^3;
    1,tend,tend^2,tend^3,tend^4,tend^5;
    0,1,2*tend,3*tend^2,4*tend^3,5*tend^4;
    0,0,2,6*tend,12*tend^2,20*tend^3];
b=[qstart,qdotstart,0,qend,qdotend,0]';
X=A\b;

end

