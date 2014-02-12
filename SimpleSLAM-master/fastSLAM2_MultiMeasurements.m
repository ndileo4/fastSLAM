%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Copyright 2010 Randolph Voorhies
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
% The number of timesteps for the simulation
timesteps = 200;

% The maximum distance from which our sensor can sense a landmark
max_read_distance = 2;

% The actual positions of the landmarks (each column is a separate landmark)
real_landmarks = [1.0,  2.0,  0.0, 0.0, 1.0;     % x
                  3.0,  2.5   3.4, 1.5, 3.5;     % y
                  0.0,  0.0   0.0, 0.0, 0.0];    % Nothing

% real_landmarks = [1.0,  2.0,  0.0;
%                   1.5,  2.5   3.5;
%                   0.0,  0.0   0.0];   

% The initial starting position of the robot
real_position = [0.0;      % x
                 -1.0;     % y
                 pi/3.0];  % rotation

% The movement command given tot he robot at each timestep                 
movement_command = [.05;     % Distance
                    .01];    % Rotation
                    
% The Gaussian variance of the movement commands
movement_variance = [.1;   % Distance
                     .05]; % Rotation
M = [movement_variance(1), 0.0;
     0.0, movement_variance(2)];

% The Gaussian variance of our sensor readings
measurement_variance = [0.1;    % Distance
                        0.01;   % Angle
                        .0001]; % Landmark Identity
R = [measurement_variance(1), 0.0, 0.0;
     0.0, measurement_variance(2), 0.0;
     0.0, 0.0, measurement_variance(3)];
 


% Create the particles and initialize them all to be in the same initial
% position. 
particles = [];
num_particles = 100;
% num_landmarks(1)=0;
for i = 1:num_particles
  particles(i).w = 1.0/num_particles;
  particles(i).position = real_position;
  for lIdx=1:size(real_landmarks,2)
    particles(i).landmarks(lIdx).seen = false;
    particles(i).landmarks(lIdx).E=zeros(3,3);
    particles(i).num_landmarks(1)=0;
    particles(i).lm_weight=1.0/num_particles;
  end
end

pos_history = [];
default_importance=0.1; 

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% SIMULATION
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for timestep = 2:timesteps
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Move the actual robot  
  real_position = moveParticle(real_position, movement_command, movement_variance);
  pos_history = [pos_history, real_position];

  % Move the actual particles
  %x_hat=g(x,u)
  for pIdx = 1:num_particles
    particles(pIdx).position = moveParticle( ...
    particles(pIdx).position, movement_command, movement_variance);
    particles(pIdx).position(3) = real_position(3);
  end

  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % Try to take a reading from each landmark
% %   doResample = false;
 
  for lIdx = 1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);

    % Take a real (noisy) measurement from the robot to the landmark
    [z_real(:,lIdx), G(:,:,lIdx)] = getMeasurement(real_position, real_landmark, measurement_variance);
    read_distance(lIdx) = z_real(1,lIdx);
    read_angle(lIdx)    = z_real(2,lIdx);

     to_delete=find(read_distance> max_read_distance);
  end
    z_real(:,to_delete)=[];
    read_angle(to_delete)=[];
    read_distance(to_delete)=[];
    G(:,:,to_delete)=[];
    

        
      for pIdx = 1:num_particles
               
          %measurementsLikelihood()
          clear Q Ex Mux z_hat residual new_residual phi
          for j=1:particles(pIdx).num_landmarks(timestep-1)                           
              [z_p(:,j), Hx(:,:,j) Hm(:,:,j)] = getMeasurement2(particles(pIdx).position, particles(pIdx).landmarks(j).pos, [0;0]);
              for measured_lm=1:size(z_real,2)
                  
              residual = z_real(:,measured_lm) - z_p(:,j);
              Q(:,:,j,measured_lm) = Hm(:,:,j) * particles(pIdx).landmarks(j).E * Hm(:,:,j)' + G(:,:,measured_lm);
              Ex(:,:,j,measured_lm) = inv(Hx(:,:,j)'*inv(Q(:,:,j,measured_lm))*Hx(:,:,j)+inv(M));
              Mux(:,:,j,measured_lm)= Ex(:,:,j,measured_lm)*Hx(:,:,j)'*inv(Q(:,:,j,measured_lm))*residual+particles(pIdx).position;
              [z_hat(:,:,j,l) Ignore1 Ignore2]= getMeasurement2(Mux(:,:,j,measured_lm), particles(pIdx).landmarks(j).pos, [0;0]);
              new_residual=z_real(:,measured_lm)-z_hat(:,:,j,l);
              phi(measured_lm,j)=norm(2*pi*Q(:,:,j,measured_lm)).^(-1/2)*exp(-1/2*(new_residual)'*inv(Q(:,:,j,measured_lm))*(new_residual));
%               particles(pIdx).lm_weight(measured_lm,j) = norm(2*pi*Q(:,:,j,measured_lm)).^(-1/2)*exp(-1/2*(new_residual)'*inv(Q(:,:,j,measured_lm))*(new_residual));
              end
          end
          
          %probably going to delete this hold on for now
%           particles(pIdx).lm_weight(particles(pIdx).num_landmarks(timestep-1)+1) = 0.09; %importance factor, new feature
           particles(pIdx).num_candidates(timestep)=particles(pIdx).num_candidates(timestep-1);

%           clear c_hat
          data_associate_mat=zeros(size(z_real,2),particles(pIdx).num_candidates(timestep-1));
          for measured_lm=1:size(z_real,2)
              if max(phi(measured_lm,:)) < default_importance
                  %particles(pIdx).num_landmarks(timestep)=particles(pIdx).num_landmarks(timestep)+1; %add new landmark
                 particles(pIdx).num_candidates(timestep)=particles(pIdx).num_candidates(timestep)+1; %add new landmark
                  data_associate_mat(measured_lm,particles(pIdx).num_candidates(timestep))=measured_lm; %set correspondance b/t measurement and feature j
                  
              else
                  [val idx]=max(phi(measured_lm,:));
                  data_associate_mat(measured_lm,idx)=measured_lm; %measurement is a known feature
                  
              end %end if 
                            
          end %end for
          data_associate_vect=max(data_associate_mat,[],1); %vector that contains measurement correspondance to landmarks
    if isempty(data_associate_vect)==1
       continue; %if there is not new measurement, skip to the next timestep 
    end
          
%           [particles(pIdx).w c_hat] = max(particles(pIdx).lm_weight); %max likelihood and index of ML feature
%           particles(pIdx).num_landmarks(timestep) = max([particles(pIdx).num_landmarks(timestep-1) c_hat]); %new number of features
%           c_hat_list(measured_lm)=c_hat;
%           j=1;
            
            %RobotPoseUpdate()
            if sum(data_associate_vect(1:particles(pIdx).num_landmarks(timestep-1)))==0
                    particles(pIdx).position = moveParticle( ...
    particles(pIdx).position, movement_command, movement_variance); %all features are previously unseen 
            else
%                 Sigmax = M; %initialize covariance
%                 MuX = particles(pIdx).position; %initialize current position
                    for j=1:particles(pIdx).num_landmarks(timestep-1)
                        if data_associate_vect(j)>0 && particles(pIdx).landmarks(j).type==1
                           if j==1%accounts for vectors starting at index 1
                               Sigmax(j)=inv(Hx(:,:,j)'*inv(Q(:,:,j,data_associate_vect(j)))*...
                                   Hx(:,:,j)+inv(M));
                               MuX(j) = particles(pIdx).position + Sigmax(j)*Hx(:,:,j)' *...
                                   inv(Q(:,:,j,data_associate_vect(j)))*...
                                   (z_real(:,data_associate_vect(j))-z_p(:,j));
                           else
                               Sigmax(j)=inv(Hx(:,:,j)'*inv(Q(:,:,j,data_associate_vect(j)))*...
                                   Hx(:,:,j)+inv(Sigmax(j-1)));
                               MuX = MuX(j-1) + Sigmax(j)*Hx(:,:,j)' *...
                                   inv(Q(:,:,j,data_associate_vect(j)))*...
                                   (z_real(:,data_associate_vect(j))-z_p(:,j));
                           end %end if
                            
                        end %end if
                    end %end for
                particles(pIdx).Ex=Sigmax(j);
                particles(pIdx).MuX=MuX(j);
                deviate = chol(particles(pIdx).Ex);
                
                particles(pIdx).position = repmat(particles(pIdx).MuX,1,1) + randn(1,2)*deviate;

                    
            end %end if
            
            %LandmarksUpdate()
            particles(pIdx).weight=1;
            for j=1:particles(pIdx).num_landmarks(timestep-1)
                if data_associate_vect(j) > 0 
                    particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter+1;
                    [z_j(:,j), Hx(:,:,j) Hm(:,:,j)] = getMeasurement2(particles(pIdx).position, particles(pIdx).landmarks(j).pos, [0;0]);
                    Q_hat(:,:,j)=Q(:,:,j,data_associate_vect(j))+...
                            Hm(:,:,j)*particles(pIdx).landmarks(j).E*Hm(:,:,j)';
                    K=particles(pIdx).landmarks(j).E*Hm(:,:,j)'*inv(Q_hat(:,:,j));
                    last_landmark_pos=particles(pIdx).landmarks(j).pos;
                    particles(pIdx).landmarks(j).pos= particles(pIdx).landmarks(j).pos+...
                                    K*(z_real(:,data_associate_vect(j))-z_j(:,j));
                                
                    particles(pIdx).landmarks(j).E=(eye(size(Hm(:,:,j)))-...
                                    K*Hm(:,:,j))*particles(pIdx).landmarks(j).E;
                                if particles(pIdx).landmarks(j).type==1
                                   [Ignore1 Hx(:,:,j) Ignore2] = getMeasurement2(particles(pIdx).position,...
                                       last_landmark_pos, [0;0]);
    
                                   L= Hx(:,:,j)*M*Hx(:,:,j)'+Q_hat(:,:,j);
                                   w_hat=(2*pi)^(-1/2)*norm(L).^(1/2)*...
                                        exp(-1/2*(z_real(:,data_associate_vect(j))-z_j(:,j)'...
                                        *inv(L)*(z_real(:,data_associate_vect(j));
                                else
                                    w_hat=1;
                                end %end if
                else
                    particles(pIdx).landmarks(j).pos=particles(pIdx).landmarks(j).pos;
                    particles(pIdx).landmarks(j).E=particles(pIdx).landmarks(j).E;
                    w_hat=1-default_importance;
                    dist_to_landmark=norm([particles(pIdx).landmarks(j).pos(1)-particles(pIdx).position(1) particles(pIdx).landmarks(j).pos(2)-particles(pIdx).position(2)]);

                    if dist_to_landmark< max_read_distance
                        particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter-1;
                    end %end if
                    
                  particles(pIdx).weight=particles(pIdx).weight*w_hat;  
                    
                end %end if
            end %end for



            i=1;
          for j=1:particles(pIdx).num_candidates(timestep) %length(data_associate_vect) 
              if (data_associate_vect(j)>0 && j>particles(pIdx).num_candidates(timestep-1))
                % If we have never seen this landmark, then we need to initialize it.
                % We'll just use whatever first reading we recieved.                
                  particles(pIdx).candidates(j).pos = [particles(pIdx).position(1) + cos(read_angle(data_associate_vect(j)))*read_distance(data_associate_vect(j));
                      particles(pIdx).position(2) + sin(read_angle(data_associate_vect(j)))*read_distance(data_associate_vect(j));
                      0];
          % Initialize the landmark position covariance
                    H(:,:,j)=G(:,:,data_associate_vect(j));
                    particles(pIdx).candidates(j).E = inv(G(:,:,data_associate_vect(j)))' * M * inv(G(:,:,data_associate_vect(j)));
                    particles(pIdx).candidates(j).counter=1; %initialize counter
                    particles(pIdx).num_candidates(timestep)=particles(pIdx).num_candidates(timestep)+1;
     
                  
              elseif (data_associate_vect(j)>0 && j<=particles(pIdx).num_candidates(timestep-1))
                  %this candidate landmark has already been seen, update
                  
                  K = particles(pIdx).landmarks(j).E * G(:,:,data_associate_vect(j)) * inv(Q(:,:,j,data_associate_vect(j)));
                  % Mix the ideal reading, and our actual reading using the Kalman gain, and use the result
                  % to predict a new landmark position
                  particles(pIdx).landmarks(j).pos = particles(pIdx).landmarks(j).pos + K*(z_real(:,data_associate_vect(j))-z_p(:,j));
                  
                  % Update the covariance of this landmark
                  particles(pIdx).landmarks(j).E = (eye(size(K)) - K*G(:,:,data_associate_vect(j))')*particles(pIdx).landmarks(j).E;
                  particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter+1; %increment counter
                  
              else

                  % We are leaving these unchanged.  These are commented
                  % out, but this is what is actually happening in alg.
%                   particles(pIdx).landmarks(j).pos = particles(pIdx).landmarks(j).pos;
%                   particles(pIdx).landmarks(j).E=particles(pIdx).landmarks(j).E;
                    
                    dist_to_landmark=norm([particles(pIdx).landmarks(j).pos(1)-particles(pIdx).position(1) particles(pIdx).landmarks(j).pos(2)-particles(pIdx).position(2)]);
                    if dist_to_landmark > max_read_distance
                            %counter(pIdx,j)=counter(pIdx,j) %don't change counter
                    else
                  particles(pIdx).landmarks(j).counter=particles(pIdx).landmarks(j).counter-1; %decrement counter                    
                       
                         if particles(pIdx).landmarks(j).counter<0 
%                              particles(pIdx).num_landmarks(timestep)= particles(pIdx).num_landmarks(timestep)-1;
%                             particles(pIdx).num_landmarks(timestep-1)= particles(pIdx).num_landmarks(timestep)-1 
%                             particles(pIdx).landmarks(j)= []; %delete feature
%                             j=j-1
                            feature_to_delete(i)=j; %we're going to delete all these features at the end of the loop
                            i=i+1;
              
%                         end
                        end
                    end 
                            
              end %if
          end %end j

          % end %else

             try 
                 feature_to_delete
              particles(pIdx).landmarks(feature_to_delete)=[];
              particles(pIdx).num_landmarks(timestep)= particles(pIdx).num_landmarks(timestep)-length(feature_to_delete);

                    
             end
                   clear feature_to_delete 


      end %pIdx
    
    
        particles = resample(particles);
        
   %distance


  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  % PLOTTING
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
  clf;
  hold on;

  % Plot the landmarks
  for lIdx=1:size(real_landmarks,2)
    plot(real_landmarks(1,lIdx), real_landmarks(2,lIdx), 'b*');
  end

%   for lIdx = 1:particles(pIdx).num_landmarks(timestep)
% %     if(particles(1).landmarks(lIdx).seen)
%       avg_landmark_guess =[0;0;0];
%       for pIdx = 1:length(particles)
%         avg_landmark_guess = avg_landmark_guess + particles(pIdx).landmarks(lIdx).pos;
%       end
%       avg_landmark_guess = avg_landmark_guess / length(particles);
%       plot(avg_landmark_guess(1), avg_landmark_guess(2), 'ko');
% %     end
%   end


      for pIdx = 1:length(particles)
        for lIdx=1:particles(pIdx).num_landmarks(timestep)
            plot(particles(pIdx).landmarks(lIdx).pos(1),particles(pIdx).landmarks(lIdx).pos(2),'b.')
        end
      end
%       avg_landmark_guess = avg_landmark_guess / length(particles);
%       plot(avg_landmark_guess(1), avg_landmark_guess(2), 'ko');
%     end
  

  % Plot the particles
  particles_pos = [particles.position];
  plot(particles_pos(1,:), particles_pos(2,:), 'r.');

  % Plot the real robot
  plot(pos_history(1,:), pos_history(2,:), 'r');
  w = .1;
  l = .3;
  x = real_position(1);
  y = real_position(2);
  t = real_position(3);
  plot(real_position(1), real_position(2), 'mo', ...
                                           'LineWidth',1.5, ...
                                           'MarkerEdgeColor','k', ...
                                           'MarkerFaceColor',[0 1 0], ...
                                           'MarkerSize',10);

  % Show the sensor measurement as an arrow
  for lIdx=1:size(real_landmarks,2)
    real_landmark = real_landmarks(:, lIdx);
%     if(read_distance(lIdx) < max_read_distance)
        try
      line([real_position(1), real_position(1)+cos(read_angle(lIdx))*read_distance(lIdx)], ...
           [real_position(2), real_position(2)+sin(read_angle(lIdx))*read_distance(lIdx)]);
        end
%         end
  end

  axis([-5, 5, -4, 7]);
  pause(.01);
end



