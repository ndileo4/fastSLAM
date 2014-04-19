clear all

% The number of timesteps for the simulation
timesteps = 350;


%Load Data from MrT Data Collect
load DATA
%[time, dtime, l_wheel desired speed, l_wheel measured speed, 
%r_wheel desired speed, r_wheel measured speed];

robot_position = [0;0;0];
robot_history = robot_position;
for i=1:length(data)
   
    robot_position = MoveMrT(robot_position, [data(i,3) data(i,5)], [0 0],data(i,2));
    
    robot_history(1:3,end+1)=robot_position;
    
    
end



plot(robot_history(1,:),robot_history(2,:),'r')
axis equal