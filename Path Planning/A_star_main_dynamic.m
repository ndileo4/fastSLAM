% Nick DiLeo
% Villanova University 2014

%INFO
% Script to create a dynamic map of a robotic environment.  You must supply
% a starting and ending "gate" to give the robot an initial starting
% position and final destination.  The size of the field is taken from the
% RobotX rules.  The environment is said to be 40m wide and a maximum of
% 100m long
close all
clear all

% Do you want to write a .gif out
write_gif = false;

%Input start gate 1,2, or 3
start_gate = 1;

%Input end gate X,Y, or Z
end_gate = 'Z';

%load config paramters into workspace
config();

% Decide our start gate - robot initial position
switch start_gate
    case 1
        robot_position = [10;0;0]; % set initial x,y,yaw
    case 2
        robot_position = [20;0;0];
    case 3
        robot_position = [30;0;0];
    otherwise
        error('Invalid Start Gate');
end

% Decide our end gate - desired final position
switch end_gate
    case 'X'
        desired_end = [10;100;0]; % set initial x,y,yaw
    case 'Y'
        desired_end = [20;100;0];
    case 'Z'
        desired_end = [30;100;0];
    otherwise
        error('Invalid End Gate');
end

iteration=1;
%% SIMULATION
seen_idx = double.empty(1,0); %create empty vector
dist_to_end = (robot_position(1)-desired_end(1))^2+(robot_position(2)+desired_end(2))^2;
obstacles=double.empty(2,0);
%keep going with simulation until we get close to goal
while dist_to_end> min_dist2wp^2
    
    %Solve A-Star path plan if there is a new obstacle
    %TODO: Ensure A-Star accounts for current vehicle heading
    if new_obstacle == 1
        [path_spline] = return_astar_plan(robot_position,...
            desired_end,obstacles,cell_resolution,obstacle_padding);
        
        waypoint_index=1; %reset the waypoint index b/c we have a new path
        
    end
    
    %to find out the movement commands that we need to take
    [yaw_command,waypoint_index]= compute_steering(robot_position, path_spline,...
        waypoint_index, min_dist2wp, robot_position(3), max_steering_rate,...
        max_steering_angle, timestep);
    if waypoint_index==0
        break; %reached our last point
    end
    
    
    %you can update the movement_command at each iteration
    movement_command = [0.1;yaw_command]; % [velocity command;yaw command]
    robot_position = updateMovement(robot_position, movement_command, movement_variance);
    
    %Find out which obstacle we can see from our current position
    new_measurement = getMeasurement(robot_position, real_obstacles);
    read_distance = new_measurement(1,:);
    read_angle = new_measurement(2,:);
    to_keep=find(min_read_angle+robot_position(3) < read_angle & read_angle < max_read_angle+robot_position(3) & read_distance < max_read_distance);
    
    
    for i=1:length(to_keep)
        if any(to_keep(i)==seen_idx)
            continue; %if we've already seen this, do nothing
        else
            seen_idx(end+1) = to_keep(i); %add new obstacle to seen obstacles
        end
        
    end
    
    last_obstacles=obstacles;
    obstacles = real_obstacles(:,seen_idx); %select obstacles that we know about
    change_obstacles=size(obstacles,2)-size(last_obstacles,2);
    
    if change_obstacles == 0 || isempty(obstacles)
        new_obstacle =0; %there are no new obstacles, we won't replan
    else
        new_obstacle =1; %there's a new obstacles, let's replan!
    end
    
    
    dist_to_end = (robot_position(1)-desired_end(1))^2+...
        (robot_position(2)+desired_end(2))^2;
    
    robot_history(:,end+1)=robot_position;
    clf;
    %PLOTTING
    hold on
    plot(obstacles(1,:),obstacles(2,:),'k*'); %plot obstacles
    plot(gate_buoys(1,1:2),gate_buoys(2,1:2),'r*'); %plot red buoy gates
    plot(gate_buoys(1,3:7),gate_buoys(2,3:7),'w*'); %plot white buoy gates
    plot(gate_buoys(1,8:9),gate_buoys(2,8:9),'g*'); %plot green buoy gates
    plot(robot_position(1),robot_position(2),'Marker','*','MarkerFaceColor',[1 0.502 0]); %plot robot's position
    plot(path_spline(1,:),path_spline(2,:),'r-')
    plot(robot_history(1,:),robot_history(2,:),'g-')
    
    
    xlabel('x (meters)');ylabel('y (meters)');
    axis equal
    axis([0, 40, -10, 110]);
    set(gca,'Color',[0.4 0.698 1])
    hold off
    
    if write_gif
        %write to a .gif
        frame = getframe(1);
        im = frame2im(frame);
        [imind,cm] = rgb2ind(im,256);
        if iteration == 1;
            imwrite(imind,cm,'testgif.gif','gif', 'Loopcount',inf);
        else
            imwrite(imind,cm,'testgif.gif','gif','WriteMode','append');
        end
    end
    
    iteration=iteration+1;
    pause(0.03)
    
end

