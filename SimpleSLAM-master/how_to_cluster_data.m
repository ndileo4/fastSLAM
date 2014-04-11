%Example of clustering all the landmark estimates returned from fastSLAM

%create a matrix of all landmark estimates estimates from each particle
tic 
row_counter=0;
% landmarks_position = [0 0 0];
for pIdx=1:num_particles
   for j=1:particles(pIdx).num_landmarks(end) %this should be index timestep when in full script
       row_counter=row_counter+1;
       landmarks_positions(row_counter,:)=particles(pIdx).landmarks(j).pos(1:2)'; 
   end
   
%    lm_from_particles=[particles(pIdx).landmarks.pos]';
%    landmarks_position(end+1:end+size(lm_from_particles,1),:)=lm_from_particles;
%    
%    
end
toc
%clusters data together to get an estimate of landmark positions
%the sensitivity value must be tuned to properly separate clusters.
%this value is dependent on how far away "clusters" are from each other
[clusters,clusterInds] = clusterData(landmarks_positions,0.3); 
clusterMeans = cellfun(@mean,clusters,'UniformOutput',false);

