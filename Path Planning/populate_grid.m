function [world_model,mid_points_x,mid_points_y] = populate_grid(obstacles,cell_resolution,obstacle_padding)

%INPUTS
% obstacles - [x;y] list off all obstacles including
% cell_resolution - how large (in meters) should each cell in the 
%                     grid be. Higher grid size means less search space
%                     but also less places to move.  Small grid size means
%                     larger search space, but can move more freely
% obstacle_padding - amt in meters that you want the actual obstacles to be
%                      padded by. This gives us a safety factor that we don't
%                      run into the obstacles


%OUTPUTS
%DESSCRIBE THE OUTPUTS!!!!


% Script to complete an occupancy grid from the data got from SLAM
% algorithm.  In this case, we are running this after running the simulation
% to creat a static or dynamic map.  For dynamic map, this should be done at 
% the end of eachtimestep.

% Goal of this is to create a cell structure that includes positions of
% landmarks and average estimate of the vehicle position


% Set min and max world size
% These are set to reflect what Robot X Environment would look like
min_world_x = 0; %meters
max_world_x = 40; %meters
min_world_y = 0; %meters
max_world_y = 120; %meters

% cell_resolution = 2.5; %meters - currently same resolution in x and y

%specifies the amount of cells around every obstacle that we should pad
amt_to_pad=ceil(obstacle_padding/cell_resolution);


% These will be the points for each cell in our "world_model"
mid_points_x = [min_world_x:cell_resolution:max_world_x];
mid_points_y = [max_world_y:-cell_resolution:min_world_y]; %since indexing runs from smallest to largest

world_model = zeros(length(mid_points_y),length(mid_points_x));

if isempty(obstacles)
    return; %just send back an empty grid if no obstacles
end
       
       for i = 1:size(obstacles,2)
       tmp = abs(mid_points_x-obstacles(1,i));
       [dummy idx1] = min(tmp);  %index of the closest value
       tmp = abs(mid_points_y-obstacles(2,i));
       [dummy idx2] = min(tmp);  %index of the closest value

       %create the indices that we want to pad
       y_vector_pad=[idx2-amt_to_pad:idx2+amt_to_pad];
       x_vector_pad=[idx1-amt_to_pad:idx1+amt_to_pad];
       
       %We want to ensure that we don't access an invalid index
       %Right now we check if any padding is outside of acceptable range.
       %If it is, we just don't pad that landmark.  There is probably a
       %nicer way to do this (e.g. check how many cells you can pad without
       %going out of bounds).
       check_y= y_vector_pad>size(world_model,1) | y_vector_pad<=0;
       check_x= x_vector_pad>size(world_model,2) | x_vector_pad<=0;
       
       if any(check_y)==0 && any(check_x)==0
       world_model(y_vector_pad,...
                    x_vector_pad)=0.8;
       end
       
       world_model(idx2,idx1)=1;
       
       end
%    end
%     
% end
% 
% [clusters,clusterInds] = clusterData(landmarks_positions,0.8);
% clusterMeans = cellfun(@mean,clusters,'UniformOutput',false);

% % % Classification using hierarchal clustering
% % a = 5*[randn(500,1)+5, randn(500,1)+5];
% % b = 5*[randn(500,1)+5, randn(500,1)-5];
% % c = 5*[randn(500,1)-5, randn(500,1)+5];
% % d = 5*[randn(500,1)-5, randn(500,1)-5];
% % e = 5*[randn(500,1), randn(500,1)];
% % all_data = [a;b;c;d;e];
% % close all
% % plot(a(:,1),a(:,2),'.');hold on
% % plot(b(:,1),b(:,2),'r.');
% % plot(c(:,1),c(:,2),'g.');
% % plot(d(:,1),d(:,2),'k.');
% % plot(e(:,1),e(:,2),'c.');
% % 
% % all_data = [a;b;c;d;e];
% % 
% % IDX = kmeans(all_data, 6);
% % for k = 1:2500
% % text(all_data(k,1),all_data(k,2),num2str(IDX(k))); hold on
% % end
% % axis([-70 70 -70 70])
% % 
% % all_data=landmarks_positions;
% % Y = pdist(all_data);
% % Z = linkage(Y);
% % threshold = 1.152;
% % T = cluster(Z,'cutoff',threshold);
% % 
% % figure();
% % for k = 1:275
% % text(all_data(k,1),all_data(k,2),num2str(T(k))); hold on
% % end
% % axis([-5 5 0 5])
% % 

% 
% particles_pos = [particles.position];
% avg_robot_position = mean(particles_pos,2); %calculate average x,y,yaw for all particles
% 


