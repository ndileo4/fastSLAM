%config file for running dynamic A Star Planner.
%This will hold all the parameters that we can modify without cluttering up
%the code

%Parameters for map building
movement_variance = [0;0]; %Zero noise velocity and rotation commands
max_read_angle = 45*pi/180; %max viewing angle
min_read_angle = -max_read_angle;
max_read_distance = 7; %3 meters max viewing distance
min_dist2wp = 1.5; %minimum distance to the current waypoint before we go onto next one
timestep = 0.1; %timestep between each iteration
max_steering_angle=45*pi/180;
max_steering_rate = 30*180/pi; %max stering rate (rad/s)
new_obstacle = 1;
robot_history=double.empty(3,0);


%Paramters for A* Search Planner
cell_resolution = 2.0; %how big (meters) each cell is in our occupancy grid-Square grid cells
obstacle_padding = 1.5; %how many meters to pad each seen obstacle


%% Set up the obstacle positions, desired start/end postitions

%These are really only used for plotting purposes
gate_buoys = [5  5  15  15  25  25   25 35  35;
              0 100 0  100   0  100  0  0 100];
          
real_obstacles = [7.1429   14.9770   24.4700   30.6452   22.5346   12.1198    8.8940  18.1106   29.5392   36.6359   29.4470   18.0184    6.0369   13.3180 26.8664   18.7558   27.3272   13.4101;
   14.7368   15.0877   17.1930   27.3684   40.0000   44.2105   62.8070   63.1579   63.8596   74.3860   80.0000   83.1579   74.3860   41.0526  17.1930   24.9123   55.0877   56.4912];
          



