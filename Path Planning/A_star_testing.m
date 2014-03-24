% Simple test to attempt to get A star to work with one obstacle

%Create a "grid" of obstacles. 0 for no obstacle. 1 fo obstacle
clear all
close all

start_gate = 1; %start gate 1,2, or 3
end_gate = 'Z'; %desired end gate - X,Y,Z
cell_resolution = 2.0;
obstacle_padding = 1;

[start_pos desired_end obstacles] = create_static_map(start_gate,end_gate);
[world_model,mid_points_x, mid_points_y] = populate_grid(obstacles,cell_resolution,obstacle_padding);

% start_pos = [2,4]; %start position x,y
% desired_end = [7, 2]; %desired end position

% obstacles = [0 0 0 0 0 0 0 0 0; %6
%              0 0 0 0 0 0 0 0 0; %5
%              0 0 0 0 1 0 0 0 0; %4
%              0 0 0 0 1 0 0 0 0; %3 
%              0 0 0 0 1 0 0 0 0; %2
%              0 0 0 0 0 0 0 0 0; %1
%              0 0 0 0 0 0 0 0 0]; %0 index
%            % 0 1 2 3 4 5 6 7 8

% obstacles = [0 0 0 0 0 0 0 0 0; %6
%              0 0 0 0 1 0 0 0 0; %5
%              0 0 0 0 1 0 0 0 0; %4
%              0 0 0 0 0 0 0 0 0; %3 
%              0 0 0 0 1 0 0 1 0; %2
%              0 0 0 0 0 1 0 0 0; %1
%              0 0 0 0 0 0 0 0 0]; %0 index
%            % 0 1 2 3 4 5 6 7 8
obstacles = world_model;
        
grid_cells = cell(size(obstacles));


x_add = [0 cell_resolution cell_resolution cell_resolution...
                0 -cell_resolution -cell_resolution -cell_resolution];
y_add = [cell_resolution cell_resolution 0 -cell_resolution ...
                -cell_resolution -cell_resolution 0 cell_resolution];

 dynamic_points = [1 5 1 5 1 5 1 5];   
% dynamic_points = [1 1 1 1 1 1 1 1]; 
            
open_list=cell(1,1); %initialize open_list as a cell structure
open_list{1,1}(1,1)=-1; %parent x
open_list{1,1}(2,1) = -1; %parent y
open_list{1,1}(3,1) = start_pos(1); %set intial x node in open list
open_list{1,1}(4,1) = start_pos(2); %set y position
open_list{1,1}(5,1) = 0; %g
open_list{1,1}(6,1) = 0;  %h
open_list{1,1}(7,1) = open_list{1,1}(5,1)+open_list{1,1}(6,1); %f score

closed_list = cell(0,1); %creat empty closed list

extract = @(C, k) cellfun(@(c)c(k), C) ; %function to extract element out of cell
solution_found = 0; %initialize our solution found flag

tic
while isempty(open_list) == 0
    f_all = extract(open_list,7); %this is where we store our "f" value for each node
    [lowest_f low_idx] = min(f_all); %find minimum f value
    q = open_list{low_idx,1}; %make our new node to explore to be q
    open_list(low_idx,:) = []; %pop q off open list
    

    %generate q's 8 successors and set parents to q
    successors=cell(8,1); %initialize as cell
    for i=1:8
        
        successors{i,1}(1) = q(3); %set parent's x position in our new node
        successors{i,1}(2) = q(4); %set parent's y position in our new node
        successors{i,1}(3) = q(3)+x_add(i); %set successors x position
        successors{i,1}(4) = q(4)+y_add(i); %set successors y position
                        
        %check to see if successor is goal
        if (successors{i,1}(3) == desired_end(1)) && (successors{i,1}(4) == desired_end(2))
            solution_found = 1; %we got to our goal
            break;
        end

        
        %check to see if successor is an obstacle or obstacle padding:
        %if it is, make the g score HUGE...if not assign g-score
               
       tmp = abs(mid_points_x-successors{i,1}(3));
       [~, idx1] = min(tmp);  %index of the closest value
       tmp = abs(mid_points_y-successors{i,1}(4));
       [~, idx2] = min(tmp);  %index of the closest value
   
%        world_model(idx2,idx1)=1;

      
     
        %note that we have to reverse y (row) indexing b/c our y axis
        %goes up and the matlab indexing scheme goes from top to bottom
%         if obstacles(size(obstacles,1)-successors{i,1}(4)/cell_resolution,successors{i,1}(3)/cell_resolution+1) == 1
         if obstacles(idx2,idx1) == 1 || obstacles(idx2,idx1) == 0.8

            successors{i,1}(5) = q(5) + 1000000; % arbitrarily large number
          else
            %SPEED UP IDEA: just leave these as squared w/o taking
            %the square root
            %this is where you can put more weight on kinematically
            %infeasible motions - the sqrt was removed - shaved several
            %seconds off
            successors{i,1}(5) = q(5) + dynamic_points(i)*((q(3)-successors{i,1}(3))^2 ...
                                            +(q(4)-successors{i,1}(4))^2);
                
        end
        
        %now, set h 
        %SPEED UP - just leave as squared instead of square root -
        %implemented on 3/17
        successors{i,1}(6) = ((desired_end(1)-successors{i,1}(3))^2 ...
                                   +(desired_end(2)-successors{i,1}(4))^2);
        
        successors{i,1}(7) = successors{i,1}(5)+successors{i,1}(6); %set f score
        
        
        successor_position = [successors{i,1}(3) successors{i,1}(4)];
        if ~isempty(open_list) %ensure we don't access invalid index
            open_list_pos = [extract(open_list,3) extract(open_list,4)]; %get open list x,y positions
            %now we need to find out if any of the successors are on the open
            %list
            idx_on_open_list = find(ismember(open_list_pos,successor_position,'rows')==1); %return idx if true, empty if false
            
            %if this is on open list in two places, pick the one w/ lower f
            %value
            if length(idx_on_open_list)>1
                [val lower_idx] =min(extract(open_list(idx_on_open_list),7));
                idx_on_open_list = idx_on_open_list(lower_idx);
            end
        
        else
            idx_on_open_list = cell(1,0);
            
        end
        
        if ~isempty(closed_list) %ensure we don't acces invalid index
            closed_list_pos = [extract(closed_list,3) extract(closed_list,4)]; %get closed list x,y positions
            idx_on_closed_list = find(ismember(closed_list_pos,successor_position,'rows')==1);
            
            %if this is on open list in two places, pick the one w/ lower f
            %value
            if length(idx_on_closed_list)>1
                [val lower_idx] =min(extract(closed_list(idx_on_closed_list),7));
                idx_on_closed_list = idx_on_closed_list(lower_idx);
            end
        else
            idx_on_closed_list = cell(1,0);
        end
        
        
        
        if ~isempty(idx_on_open_list) %this position has appeared on open list
            if open_list{idx_on_open_list}(7) <= successors{i,1}(7)
                %then the f value on the open list is lower than the
                %f score by the proposed successor
                % so continue onto the next successor w/o adding it to
                % open list
                continue;                
            end
        end
        
        if ~isempty(idx_on_closed_list) %this position has appeared on open list
            if closed_list{idx_on_closed_list}(7) <= successors{i,1}(7)
                %then the f value on the closed list is lower than the
                %f score by the proposed successor
                % so continue onto the next successor w/o adding it to
                % open list
                continue;
            end
        end
        
        %If we get past those two continue's we'll add this successer
        %to the open list 
        open_list{end+1,1} = successors{i,1};
    end %end for each successor
                               
    closed_list{end+1,1} = q'; %push q to the closed list                           
                               

    if solution_found == 1
       %were done!
        break; 
    end

end  
toc

final_parents = [extract(closed_list,1) extract(closed_list,2)];
final_xy = [extract(closed_list,3) extract(closed_list,4)];


%reconsruct the final path
parent_idx = length(closed_list); %start at the last node
final_path = [closed_list{end,1}(3) closed_list{end,1}(4)]; %set the first entry in final path
while final_path(end,1) ~= -1
  parent_idx = find(ismember(final_xy,final_parents(parent_idx,:),'rows')==1);
  if isempty(parent_idx)
    %got to the beginning
      break;
  end
      final_path(end+1,1:2) = [final_xy(parent_idx(1),1) final_xy(parent_idx(1),2)]; 
    
end



%PLOTTING
figure(1)
% hold on
% plot(final_path(:,1),final_path(:,2),'r-')
% hold off

%  pcolor(obstacles);figure(gcf);
%  hold on
%  pcolor(start_pos(1),start_pos(2),1)

start_pos_plot = zeros(size(obstacles));
tmp = abs(mid_points_x-start_pos(1));
[dummy idx1] = min(tmp);  %index of the closest value
tmp = abs(mid_points_y-start_pos(2));
[dummy idx2] = min(tmp);  %index of the closest value
start_pos_plot(idx2,idx1)=0.4;
% start_pos_plot=padarray(start_pos_plot,[1 1],'pre');

end_pos_plot = zeros(size(obstacles));
start_pos_plot = zeros(size(obstacles));
tmp = abs(mid_points_x-desired_end(1));
[dummy idx1] = min(tmp);  %index of the closest value
tmp = abs(mid_points_y-desired_end(2));
[dummy idx2] = min(tmp);  %index of the closest value
end_pos_plot(idx2,idx1)=0.8;
% end_pos_plot=padarray(end_pos_plot,[1 1],'pre');

path_pos_plot = zeros(size(obstacles));
for i=1:length(final_path)
start_pos_plot = zeros(size(obstacles));
tmp = abs(mid_points_x-final_path(i,1));
[dummy idx1] = min(tmp);  %index of the closest value
tmp = abs(mid_points_y-final_path(i,2));
[dummy idx2] = min(tmp);  %index of the closest value  
    
path_pos_plot(idx2,idx1)=0.3;
end
% path_pos_plot=padarray(path_pos_plot,[1 1],'pre');
% 
% obstacles_plot=padarray(obstacles,[1 0],'post');
% obstacles_plot=padarray(obstacles_plot,[0 1],'post');

%close all
% figure;
% 
% view(2);
% hold on;
% surf(start_pos_plot,'EdgeColor',[0 0 0]);
% surf(end_pos_plot,'EdgeColor',[0 0 0]);
% surf(path_pos_plot,'EdgeColor',[0 0 0]);
% surf(obstacles_plot,'EdgeColor',[0 0 0]);
figure()

final_result =start_pos_plot+end_pos_plot+path_pos_plot+obstacles;
pcolor(final_result)
set(gca,'YDir','reverse');
% view(2);

final_path=flipud(final_path); %flip so we start from beginning
final_path(end+1,:)=desired_end(1:2)';
%create spline
t=[0:length(final_path)-1];
ts=[0:0.25:length(final_path)-1];
the_spline=spline(t,final_path',ts);
figure(1)
hold on
plot(the_spline(1,:),the_spline(2,:),'g-')