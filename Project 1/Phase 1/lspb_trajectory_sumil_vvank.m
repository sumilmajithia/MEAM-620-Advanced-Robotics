function X = lspb_trajectory_sumil_vvank(t, tstart, tend, qstart, qend, ~, ~)
% LSPB interpolation.

% By default, do linear interpolation.  You must replace this with the
% correct trajectory type.
tb=tstart+0.1*(tend-tstart);
tc=tstart+0.9*(tend-tstart);
A=[ 1,      tstart,    tstart^2,   0,      0,      0,      0,      0;
    0,      1,         2*tstart,   0,      0,      0,      0,      0;
    1,      tb,        tb^2,       -1,     -tb,     0,      0,      0;
    0,      1,         2*tb,       0,      -1,     0,      0,      0;
    0,      0,         0,          -1,     -tc,    1,      tc,     tc^2; 
    0,      0,         0,          0,      -1,     0,      1,      2*tc;
    0,      0,         0,          0,      0,      1,      tend,   tend^2;
    0,      0,         0,          0,      0,      0,      1,      2*tend];
b=[qstart,0,0,0,0,0,qend,0]';
X=A\b;

if( t>=tstart && t<tb)
q = X(1)+X(2)*t+X(3)*t^2;
elseif( t>=tb && t< tc)
        q = X(4)+X(5)*t;
    else
        q = X(6)+X(7)*t+X(8)*t^2;
end
end
