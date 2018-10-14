# Robot Navigation using the Attractor Dynamics Approach

This project demonstrates that the dynamic system approach can be used to generate
collision-free paths toward targets while avoid moving obstacles even if low-level sensory
information (proximity sensors in this case) is used instead of representations of the
environment, see (Althaus et al., 2001) or (Bicho et al., 1998) in /docs folder for more information.

## Dynamical Obstacles Avoidance and Targets Acquisition

The dynamics generate a heading direction for a mobile robot that is
moving successively but in a random order toward three predefined  targets
while avoiding randomly moving obstacles.

## The simulation robot used:

* ePuck

## The versions of simulation softwares used:

* Matlab 8.5.0.197613 (R2015a) 
* V-REP PRO EDU version 3.5.0

## To run the simulation:

1. Start the V-REP simulator and open the simulation scene provided (epuck-dyn-obs-tar.ttt)
2. Start Matlab and change to simulation files folder or add its path
3. Start simulation with: 

```
>> moveEpuck
```
 runs the simulation without plotting the dynamics of the heading direction
```
>> moveEpuck(1)
```
runs the simulation and plots the dynamics of the heading direction

4. If necessary, change the parameters of the dynamics and rerun the simulation  