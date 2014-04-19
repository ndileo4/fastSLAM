%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Nick DiLeo 2014
%  Villanova University

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   
function newpos = MoveMrT(pos, movement, variance,dt)
% Compute how the MrT robot should move from "pos" given the requested movement and
% some Gaussian random noise using a very simple motion model. This method is
% used to move the each of the particles.


  
  % Add some Gaussian random noise to the movement. Note that we are using
  % a smaller variance in this Gaussian distribution, as the algorithm seems
  % to work better when it underestimates the quality of the robot plant. 
  l_wheel_des    = normrnd(movement(1), variance(1));
  r_wheel_des = normrnd(movement(2), variance(2));
   wheel_radius = 0.04445; %m 
%  wheel_radius = 0.06; %m 
  wheel_base = 0.3175; %m
  
  %wheel velocities
  vel_right = r_wheel_des*wheel_radius;
  vel_left = l_wheel_des*wheel_radius;
  
  v_robot = (vel_right+vel_left)/2; %forward velocity of robot m/s
  omega_robot = (vel_left-vel_right)/wheel_base; %angular velocity of robot
  
  x_dot = sin(pos(3))*v_robot;
  y_dot = cos(pos(3))*v_robot;
  
  
  delta = zeros(3,1);
%   delta(1,1) = cos(pos(3)+rotation)*speed;
%   delta(2,1) = sin(pos(3)+rotation)*speed;
%   delta(3,1) = rotation;

    delta(1,1) = x_dot*dt;
    delta(2,1) = y_dot*dt;
    delta(3,1) = omega_robot*dt;

  newpos = pos+delta;
end
