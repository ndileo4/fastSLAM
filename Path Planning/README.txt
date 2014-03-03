There are two main files in this folder:
1) create_static_map.m
Creates one map of the positions of all landmarks,robot's start postion,
and robot's desired end position.  The start and end positions can be
changed by supplying the start_gate (1,2, or 3) and end_gate (X,Y, or Z).
Landmark positions can be changed. At the time of this writing, there are
18 obstacles in the water.  Each obstacles has only a single point, but no
size.  It would be good to assume the actual size of each obstacles is
approximately 1m x 1m

2) create_dynamic_map.m
Creates a map that gets updated at each timestep as the robot moves through
its environment.  Again, you can set start and end positions, but you will
also have to supply movement commands (velocity and heading) for
each timestep.  After each timestep, you will have a list of all obstacles
that we've seen seen already.  You can also adjust the max/min read 
angle as well as the max read distance.  These parameters tell what the
boat can see at the current iteration.
