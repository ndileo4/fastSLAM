function [robot_position end_position gates_and_obstacles] = create_static_map(start_gate,end_gate) 
%OUTPUT
% robot_pos = start position of the robot
% end_pos = desired end position of the robot
% gates_and_obstacles = one variable to contain the x,y location of all 
%                       obstacles and buoys
    
%INFO
% Script to create a static map of a robotic environment.  You must supply
% a starting and ending "gate" to give the robot an initial starting
% position and final destination.  The size of the field is taken from the
% RobotX rules.  The environment is said to be 40m wide and a maximum of
% 100m long 


% clear all;
% close all;

%Input start gate 1,2, or 3
% % start_gate = 1;
% % end_gate = 'Z';


%Input end gate X,Y, or Z
%These are really only used for plotting purposes
gate_buoys = [5  5  15  15  25  25   25 35  35;
              10 110 10  110   10  110  10  10 110];
          
obstacles = [7.1429   14.9770   24.4700   30.6452   22.5346   12.1198    8.8940  18.1106   29.5392   36.6359   29.4470   18.0184    6.0369   13.3180 26.8664   18.7558   27.3272   13.4101;
   24.7368   25.0877   27.1930   37.3684   50.0000   54.2105   72.8070   73.1579   73.8596   84.3860   90.0000   93.1579   84.3860   51.0526  27.1930   34.9123   65.0877   66.4912];
          
% Decide our start gate - robot initial position
switch start_gate
    case 1
        robot_position = [10;10;0]; % set initial x,y,yaw
    case 2
        robot_position = [20;10;0]; 
    case 3
        robot_position = [30;10;0];
    otherwise
        error('Invalid Start Gate');
end

% Decide our end gate - desired final position
switch end_gate
    case 'X'
        end_position = [10;110;0]; % set initial x,y,yaw
    case 'Y'
        end_position = [20;110;0]; 
    case 'Z'
        end_position = [30;110;0];
    otherwise
        error('Invalid End Gate');
end


%PLOTTING          
hold on          
plot(obstacles(1,:),obstacles(2,:),'k*'); %plot obstacles
plot(gate_buoys(1,1:2),gate_buoys(2,1:2),'r*'); %plot red buoy gates
plot(gate_buoys(1,3:7),gate_buoys(2,3:7),'w*'); %plot white buoy gates
plot(gate_buoys(1,8:9),gate_buoys(2,8:9),'g*'); %plot green buoy gates
plot(robot_position(1),robot_position(2),'Marker','*','MarkerFaceColor',[1 0.502 0]); %plot robot's position
xlabel('x (meters)');ylabel('y (meters)');
axis equal
axis([0, 40, 0, 120]);
set(gca,'Color',[0.4 0.698 1]) 

%combine gates and obstacles into one variable
gates_and_obstacles = [obstacles, gate_buoys];
          