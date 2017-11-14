function [desired_state] = diamond(t, qn)
% DIAMOND trajectory generator for a diamond

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
T=20;
if t<= T/4
pos = [(t/T); (t/(T/4))*sqrt(2); (t/(T/4))*sqrt(2)];
vel = [1/T; sqrt(2)/(T/4); sqrt(2)/(T/4)];
acc = [0; 0; 0];
end
if t> T/4 && t<=T/2
pos = [t/T; ((-sqrt(2))*(t-T/2))/(T/4); (t*sqrt(2))/(T/4)];
vel = [1/T; (-sqrt(2))/(T/4); sqrt(2)/(T/4)];
acc = [0; 0; 0];
end
if t> T/2 && t <=(3*T)/4
pos = [t/T; ((t*(-sqrt(2)))/(T/4))+2*sqrt(2); ((t*(-sqrt(2)))/(T/4))+4*sqrt(2)];
vel = [1/T; (-sqrt(2))/(T/4); (-sqrt(2))/(T/4)];
acc = [0; 0; 0];
end
if t > (3*T)/4 && t<=T
pos = [t/T; (t*(sqrt(2)))/(T/4)-4*sqrt(2); (-t*(sqrt(2)))/(T/4)+4*sqrt(2)];
vel = [1/T; (sqrt(2))/(T/4); -(sqrt(2))/(T/4)];
acc = [0; 0; 0];
end


if t>T
  pos = [1; 0; 0];
vel = [0; 0;0];
acc = [0; 0; 0];
end
    
    yaw = 0;
yawdot = 0;

% =================== Your code ends here ===================

desired_state.pos = pos(:);
desired_state.vel = vel(:);
desired_state.acc = acc(:);
desired_state.yaw = yaw;
desired_state.yawdot = yawdot;

end
