function [desired_state] = circle(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
if t<=15
w=2*pi/15;
pos = [5*cos(w*t); 5*sin(w*t); (2.5/15)*t];
vel = [-5*w*sin(w*t); 5*w*cos(w*t); 2.5/15];
acc = [-5*w^2*cos(w*t); -5*w^2*sin(w*t);0];
yaw = 0;
yawdot = 0;
else
    pos = [5; 0; 2.5];
vel = [0; 0; 0];
acc = [0; 0;0];
yaw = 0;
yawdot = 0;
end

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
