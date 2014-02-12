%plotting all landmarks

figure(2)
hold on

for i=1:length(particles)
   for lIdx=1:particles(i).num_landmarks(end)
      if lIdx==1
         plot(particles(i).landmarks(lIdx).pos(1),particles(i).landmarks(lIdx).pos(2),'b.')
      elseif lIdx==2
              plot(particles(i).landmarks(lIdx).pos(1),particles(i).landmarks(lIdx).pos(2),'r.')
      elseif lIdx==3
          plot(particles(i).landmarks(lIdx).pos(1),particles(i).landmarks(lIdx).pos(2),'c.')
          
      elseif lIdx==4
         plot(particles(i).landmarks(lIdx).pos(1),particles(i).landmarks(lIdx).pos(2),'g.')
                    
      end
   end
    
    
end



