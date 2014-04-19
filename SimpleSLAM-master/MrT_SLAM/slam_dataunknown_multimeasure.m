%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Copyright 2010 Randolph Voorhies
%  Edited 2014 Nick DiLeo - Modifications for unknown data association 
%  and multiple measurements in a single timestep

%  This program is free software: you can redistribute it and/or modify
%  it under the terms of the GNU General Public License as published by
%  the Free Software Foundation, either version 3 of the License, or
%  (at your option) any later version.
%
%  This program is distributed in the hope that it will be useful,
%  but WITHOUT ANY WARRANTY; without even the implied warranty of
%  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
%  GNU General Public License for more details.
%
%  You should have received a copy of the GNU General Public License
%  along with this program.  If not, see <http://www.gnu.org/licenses/>.
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%   


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% PARAMETERS
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
% The number of timesteps for the simulation
timesteps = 350;

%Load image info - just to see what robot is seeing
load Image_Info

%Load Data from MrT Data Collect
load DATA
data(1:5,4)=0;
data(1:5,6)=0;
%[time, dtime, r_wheel desired speed, r_wheel measured speed, 
%l_wheel desired speed, l_wheel measured speed];

load LANDMARK_DATA
%[angles distances]

% The maximum distance from which our sensor can sense a landmark
max_read_distance = 5;
min_read_distance = 0.3;
min_read_angle = -22*pi/180; %radians- maximum read angle
max_read_angle = 22*pi/180;

% The initial starting position of the robot
avg_robot_position = [0;      % x
                 0;     % y
                 0];  % rotation             

                    
% The Gaussian variance of the movement commands
movement_variance = [.1;   % Left wheel
                     .1]; % Right_wheel
M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
% measurement_variance = [0.01;    % Distance
%                         0.1;   % Angle
%                         .0000000001]; % Landmark Identity
% R = [measurement_variance(1), 0.0, 0.0;
%      0.0, measurement_variance(2), 0.0];
%      0.0, 0.0, measurement_variance(3)];

measurement_variance = [0.01;    % Distance
                        0.14];   % Angle

R = [measurement_variance(1), 0.0;
     0.0, measurement_variance(2)];

 


 % Create the particles and initialize them all to be in the same initial
 % position.
 particles = [];
 num_particles = 50;
 % num_landmarks(1)=0;
 for i = 1:num_particles
     particles(i).w = 1.0/num_particles;
     particles(i).position = avg_robot_position;
     
         particles(i).num_landmarks(1)=0;
         particles(i).lm_weight=1.0/num_particles;
     
 end
avg_robot_position = mean([particles.position],2);
pos_history = [];
default_importance=0.000000001; 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for timestep = 2:timesteps

  % Move the actual particles
  for pIdx = 1:num_particles
    particles(pIdx).position = MoveMrT(avg_robot_position, [data(timestep,3) data(timestep,5)],...
        movement_variance,data(timestep,2));
   %  particles(pIdx).position(3) = avg_robot_position(3); - get from compass
  end

    %just for visualization purposes
    circle_idx=sum(all_radii(:,:,timestep)>0);
    figure(2)
    imshow(gray_images(:,:,timestep));viscircles(all_centers(1:circle_idx,:,timestep),all_radii(1:circle_idx,:,timestep))

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Try to take a reading from each landmark
  doResample = false;
 clear z_real G read_distance read_angle
  for lIdx = 1:sum(any(collected_data(:,:,timestep),2)) %amount of visible landmarks
    real_landmark = collected_data(lIdx,:,timestep);

    % Take a real (noisy) measurement from the robot to the landmark
    [z_real(:,lIdx), G(:,:,lIdx)] = getRealMeasurement(avg_robot_position, real_landmark, [0 0]);
    read_distance(lIdx) = z_real(1,lIdx);
    read_angle(lIdx)    = z_real(2,lIdx);

  end
  
%     %delete any features that are out of our perceptual range/angle
%     to_keep=find(min_read_angle < read_angle & read_angle < max_read_angle & read_distance < max_read_distance);
% 
%     z_real=z_real(:,to_keep);
%     read_angle=read_angle(to_keep);
%     read_distance=read_distance(to_keep);
%     G=G(:,:,to_keep);
    


  if isempty(z_real)==0
      doResample=true; %if there is not new measurement, skip to the next timestep
  end    
    
    
      for pIdx = 1:num_particles               
          
          particles(pIdx).lm_weight=[];
          for j=1:particles(pIdx).num_landmarks(timestep-1)
                
              [z_p(:,j), H(:,:,j)] = getMeasurement(particles(pIdx).position, particles(pIdx).landmarks(j).pos, [0 0]);
              for measured_lm=1:size(z_real,2) 
              residual = z_real(:,measured_lm) - z_p(:,j);
              Q(:,:,j) = H(:,:,j)' * particles(pIdx).landmarks(j).E * H(:,:,j) + R;             
              particles(pIdx).lm_weight(measured_lm,j) = particles(pIdx).w * norm(2*pi*Q(:,:,j)).^(-1/2)*exp(-1/2*(residual)'*inv(Q(:,:,j))*(residual));
              end
          end
          %probably going to delete this hold on for now
          %           particles(pIdx).lm_weight(particles(pIdx).num_landmarks(timestep-1)+1) = 0.09; %importance factor, new feature
          particles(pIdx).num_landmarks(timestep)=particles(pIdx).num_landmarks(timestep-1);
          %           clear c_hat
          
          if isempty(z_real)==1
              continue; %if there is not new measurement, skip to the next timestep
          end
          
          
          clear cost_table
          
          cost_table(1:size(z_real,2),1:particles(pIdx).num_landmarks(timestep-1))=particles(pIdx).lm_weight;
          cost_table(1:size(z_real,2),particles(pIdx).num_landmarks(timestep-1)+1 ...
              :particles(pIdx).num_landmarks(timestep-1)+size(z_real,2)) ...
              =default_importance*eye(size(z_real,2));
          particles(pIdx).w=max(max(cost_table));
          [~, idx_hold]=max(cost_table,[],1); %this is the max correseponding for each measurement
          for i=1:size(cost_table,2)
             idx_zero=setdiff([1:size(cost_table,1)],idx_hold(i));
             cost_table(idx_zero,i)=0;
          end
          
          data_associate_mat=zeros(size(z_real,2),particles(pIdx).num_landmarks(timestep));
          for measured_lm=1:size(z_real,2)
              [val IDX] = max(cost_table(measured_lm,:));
              if IDX > particles(pIdx).num_landmarks(timestep-1)
                  %we haven't seen this landmark before
                  particles(pIdx).num_landmarks(timestep)=particles(pIdx).num_landmarks(timestep)+1; %add new landmark
                  data_associate_mat(measured_lm,particles(pIdx).num_landmarks(timestep))=measured_lm; %set correspondance b/t measurement and feature j
                  
              else
                  %we've already seen this landmark
%                   [val idx]=max(particles(pIdx).lm_weight(measured_lm,:));
                  data_associate_mat(measured_lm,IDX)=measured_lm; %measurement is a known feature
                  
              end
                            
          end
          data_associate_vect=max(data_associate_mat,[],1); %vector that contains measurement correspondance to landmarks
          
          i=1;
          for j=1:particles(pIdx).num_landmarks(timestep) %length(data_associate_vect) 
              if (data_associate_vect(j)>0 && j>particles(pIdx).num_landmarks(timestep-1))
                % If we have never seen this landmark, then we need to initialize it.
                % We'll just use whatever first reading we recieved.                
                  particles(pIdx).landmarks(j).pos = [particles(pIdx).position(1) + ...
                      sin(read_angle(data_associate_vect(j))+particles(pIdx).position(3))*read_distance(data_associate_vect(j));...
                      particles(pIdx).position(2) + cos(read_angle(data_associate_vect(j))+particles(pIdx).position(3))*read_distance(data_associate_vect(j))];
                      
          % Initialize the landmark position covariance
                    H(:,:,j)=G(:,:,data_associate_vect(j));
                    particles(pIdx).landmarks(j).E = inv(G(:,:,data_associate_vect(j)))' * R * inv(G(:,:,data_associate_vect(j)));
                    particles(pIdx).landmarks(j).counter=1; %initialize counter
     
                  
              elseif (data_associate_vect(j)>0 && j<=particles(pIdx).num_landmarks(timestep-1))
                                                   
                  K = particles(pIdx).landmarks(j).E * H(:,:,j) * inv(Q(:,:,j));
%                   K = 0.1*K;
                  % Mix the ideal reading, and our actual reading using the Kalman gain, and use the result
                  % to predict a new landmark position
                  if pIdx ==50
                      if j==3
                          particles(pIdx).landmarks(j).pos
%                           K
                          (z_real(:,data_associate_vect(j))-z_p(:,j))
                      end
                  end
                  
                  particles(pIdx).landmarks(j).pos = particles(pIdx).landmarks(j).pos + K*(z_real(:,data_associate_vect(j))-z_p(:,j));
                                    
                  
                  % Update the covariance of this landmark
                  particles(pIdx).landmarks(j).E = (eye(size(H(:,:,j))) - K*H(:,:,j)')*particles(pIdx).landmarks(j).E;
                  particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter+1; %increment counter
                  
              else

                  % We are leaving these unchanged.  These are commented
                  % out, but this is what is actually happening in alg.
%                   particles(pIdx).landmarks(j).pos = particles(pIdx).landmarks(j).pos;
%                   particles(pIdx).landmarks(j).E=particles(pIdx).landmarks(j).E;
                    
                    dist_to_landmark=norm([particles(pIdx).landmarks(j).pos(1)-particles(pIdx).position(1) particles(pIdx).landmarks(j).pos(2)-particles(pIdx).position(2)]);

                    
                    ang_to_landmark=atan2((particles(pIdx).landmarks(j).pos(1) - particles(pIdx).position(1)),...
                                (particles(pIdx).landmarks(j).pos(2) - particles(pIdx).position(2)))-particles(pIdx).position(3);
                    if (dist_to_landmark > max_read_distance ||dist_to_landmark < min_read_distance || ang_to_landmark > max_read_angle || ang_to_landmark < min_read_angle)
                            %counter(pIdx,j)=counter(pIdx,j) %don't change counter
                    else
                  particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter-1; %decrement counter                    
                       
                         if particles(pIdx).landmarks(j).counter<0 

                            feature_to_delete(i)=j; %we're going to delete all these features at the end of the loop
                            i=i+1;
              
%                         end
                        end
                    end 
                            
              end %if
          end %end j

          % end %else

             try 
                 feature_to_delete;
              particles(pIdx).landmarks(feature_to_delete)=[];
              particles(pIdx).num_landmarks(timestep)= particles(pIdx).num_landmarks(timestep)-length(feature_to_delete);

                    
             end
                   clear feature_to_delete 
        
        
      end %pIdx
    
            if doResample==true
                 particles = resample(particles);               
%                particles= resample_particles(particles, NEFFECTIVE, doResample);            

            end
           
            %Cluster ladnmarks together
            row_counter=0;            
            for pIdx=1:num_particles
                for j=1:particles(pIdx).num_landmarks(end) %this should be index timestep when in full script
                    row_counter=row_counter+1;
                    landmarks_positions(row_counter,:)=particles(pIdx).landmarks(j).pos(1:2)';
                end
            end
            %clusters data together to get an estimate of landmark positions
            %the sensitivity value must be tuned to properly separate clusters.
            %this value is dependent on how far away "clusters" are from each other
            [clusters,clusterInds] = clusterData(landmarks_positions,0.3);
            clusterMeans = cellfun(@mean,clusters,'UniformOutput',false);
            
            
            
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % PLOTTING
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  figure(1)
  clf;
  hold on;

  % Plot the real landmarks
%   for lIdx=1:size(real_landmarks,2)
%       plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
%   end
  
  % Plot all particle estimates of landmarks
  for pIdx = 1:length(particles)
      for lIdx=1:particles(pIdx).num_landmarks(timestep)
          plot(particles(pIdx).landmarks(lIdx).pos(1),particles(pIdx).landmarks(lIdx).pos(2),'b.')
      end
  end
  

  % Plot the particles
  particles_pos = [particles.position];
  plot(particles_pos(1,:), particles_pos(2,:), 'r.');
  axis equal
  
  % Plot the real robot
  avg_robot_position=mean([particles.position],2);
  pos_history(1:3,end+1)=avg_robot_position;
  plot(pos_history(1,:), pos_history(2,:), 'r');
  w = .1;
  l = .3;
  x = avg_robot_position(1);
  y = avg_robot_position(2);
  t = avg_robot_position(3);
  plot(avg_robot_position(1), avg_robot_position(2), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',[0 1 0], ...
                                           'MarkerSize',5);
                                       
  %Make a carrot ^ indicating the orientation of the robot
  %rot_matrix = [cos(avg_robot_position(3)+pi/2)) -sin(avg_robot_position(3)+pi/2);sin(avg_robot_position(3)+pi/2) cos(avg_robot_position(3)+pi/2)];
  pt1=[avg_robot_position(1)-0.25;avg_robot_position(2)];
  pt2=[avg_robot_position(1);avg_robot_position(2)+0.25];
  pt3=[avg_robot_position(1)+0.25;avg_robot_position(2)];
 
  pt1=[avg_robot_position(1)+(pt1(1)-avg_robot_position(1))*cos(avg_robot_position(3))+(pt1(2)-avg_robot_position(2))*sin(avg_robot_position(3));...
      avg_robot_position(2) - (pt1(1)-avg_robot_position(1))*sin(avg_robot_position(3)) + (pt1(2)-avg_robot_position(2))*cos(avg_robot_position(3))];
  pt2=[avg_robot_position(1)+(pt2(1)-avg_robot_position(1))*cos(avg_robot_position(3))+(pt2(2)-avg_robot_position(2))*sin(avg_robot_position(3));...
      avg_robot_position(2) - (pt2(1)-avg_robot_position(1))*sin(avg_robot_position(3)) + (pt2(2)-avg_robot_position(2))*cos(avg_robot_position(3))]; 
  pt3=[avg_robot_position(1)+(pt3(1)-avg_robot_position(1))*cos(avg_robot_position(3))+(pt3(2)-avg_robot_position(2))*sin(avg_robot_position(3));...
      avg_robot_position(2) - (pt3(1)-avg_robot_position(1))*sin(avg_robot_position(3)) + (pt3(2)-avg_robot_position(2))*cos(avg_robot_position(3))]; 
   
  line([pt1(1) pt2(1)],[pt1(2) pt2(2)],'Color','g');
   line([pt2(1) pt3(1)],[pt2(2) pt3(2)],'Color','g');
  
   
  % Show the sensor measurement as an arrow
  for lIdx=1:sum(any(collected_data(:,:,timestep),2)) %amount of visible landmarks
   % real_landmark = real_landmarks(:, lIdx);
%     if(read_distance(lIdx) < max_read_distance)
        try
      line([avg_robot_position(1), avg_robot_position(1)+sin(read_angle(lIdx)+avg_robot_position(3))*read_distance(lIdx)], ...
           [avg_robot_position(2), avg_robot_position(2)+cos(read_angle(lIdx)+avg_robot_position(3))*read_distance(lIdx)]);
        end
%         end
  end
  
  timestep
%   if particles(pIdx).num_landmarks(end)==6
%      break; 
%   end
  axis ([-1 4 -1 6])
  pause(.01);
end


