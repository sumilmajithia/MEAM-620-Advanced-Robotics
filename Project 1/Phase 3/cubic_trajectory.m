function X = cubic_trajectory( tstart, tend, qstart, qend, qdotstart, qdotend)
% Cubic interpolation.

% By default, do linear interpolation.  You must replace this with the
% correct trajectory type.  Be sure to use the qdotstart and qdotend
% arguments (starting and ending velocities).
A=[1,tstart,tstart^2,tstart^3;
    0,1,2*tstart,3*tstart^2;
    1,tend,tend^2,tend^3;
    0,1,2*tend,3*tend^2];
b=[qstart,qdotstart,qend,qdotend]';
X=A\b;
end