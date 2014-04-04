Nick DiLeo
Villanova University 2014

INFO
__________________________________________________
This group of scripts set out to simulate and solve the fastSLAM problem.
This is a particle filter based approach of the general SLAM problem.

Much of the original code was written by Randolph Voorhies
(https://github.com/randvoorhies/SimpleSLAM) for the fastSLAM known data
association problem (i.e. you know which measurements correspond to which
landmarks).  The major modifications included making the scripts usable for
the unknown data association problem with multiple measurements.  This
included taking multiple measurements at one time and using a most likely
(ML) based approach to matching the new measurement with landmarks already
in the map.

An important addition to this simulation was the idea of clustering landmarks
together.  Since each particle holds its own estimates about the number and
location of landmarks, we must "cluster" together all landmark estimates to
get one single map with landmark and robot pose estimates.


USAGE
___________________________________________________
Feel free to make any modifications to these scripts as necessary for your
application.

To run the simulation:
*Pull the whole githut repo (this gives you path planning abilities as well)
*RUN slam_dataunkown_multimeasure.m - This is the main script and will run
through a simulated environment.




