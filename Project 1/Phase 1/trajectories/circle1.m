function [desired_state] = circle1(t, qn)
% CIRCLE trajectory generator for a circle

% =================== Your code goes here ===================
% You have to set the pos, vel, acc, yaw and yawdot variables
T=11;
w=2*pi/T;
if t<=T
    if t<=0.5
        pos = [5*cos(w*t); 5*sin(w*t); (2.5/T)*t];
        vel = (t/0.5)*[-5*w*sin(w*t); 5*w*cos(w*t); 2.5/T];
        acc = (1/0.5)*[-5*w^2*cos(w*t); -5*w^2*sin(w*t);0];
    elseif t>0.5 &&t<=(T-0.5)
        pos = [5*cos(w*t); 5*sin(w*t); (2.5/T)*t];
        vel = [-5*w*sin(w*t); 5*w*cos(w*t); 2.5/T];
        acc = [-5*w^2*cos(w*t); -5*w^2*sin(w*t);0];
   
        
    elseif t>(T-0.5)
        pos = [5*cos(w*t); 5*sin(w*t); (2.5/T)*t];
        vel = (1-(t-(T-0.5))/(0.5))*[-5*w*sin(w*t); 5*w*cos(w*t); 2.5/T];
        acc = (-1/0.5)*[-5*w^2*cos(w*t); -5*w^2*sin(w*t);0];
    end

else
pos = [5; 0; 2.5];
vel = [0; 0; 0];
acc = [0; 0;0];

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
