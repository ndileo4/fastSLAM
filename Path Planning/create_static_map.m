% Script to create a static map of a robotic environment.  You must supply
% a starting and ending "gate" to give the robot an initial starting
% position and final destination.  The size of the field is taken from the
% RobotX rules.  The environment is said to be 40m wide and a maximum of
% 100m long
clear all;
close all;
%Input start gate 1,2, or 3
start_gate = 1;

%Input end gate X,Y, or Z
end_gate = 'X';

%These are really only used for plotting purposes
gate_buoys = [5  5  15  15  25  25   25 35  35;
              0 100 0  100   0  100  0  0 100];
          
obstacles = [7.1429   14.9770   24.4700   30.6452   22.5346   12.1198    8.8940  18.1106   29.5392   36.6359   29.4470   18.0184    6.0369   13.3180 26.8664   18.7558   27.3272   13.4101;
   14.7368   15.0877   17.1930   27.3684   40.0000   44.2105   62.8070   63.1579   63.8596   74.3860   80.0000   83.1579   74.3860   41.0526  17.1930   24.9123   55.0877   56.4912];
          
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
        end_position = [10;100;0]; % set initial x,y,yaw
    case 'Y'
        end_position = [20;100;0]; 
    case 'Z'
        end_position = [30;100;0];
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
axis([0, 40, -10, 110]);
set(gca,'Color',[0.4 0.698 1])       
          