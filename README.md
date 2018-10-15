# Robot Navigation using the Attractor Dynamics Approach

This project demonstrates that the dynamic system approach can be used to generate
collision-free paths toward targets while avoiding moving obstacles even if low-level sensory
information (proximity sensors in this case) is used instead of representations of the
environment, see (Althaus et al., 2001) or (Bicho et al., 1998) in [docs](docs/) folder for more information

## Dynamical Obstacles Avoidance and Targets Acquisition

The dynamics generate a heading direction for a mobile robot that is
moving successively but in a random order toward three predefined  targets
while avoiding randomly moving obstacles

<img src="/pics/simulation-scene.png" alt="alt text" width="570">

Video of the simulation can be seen seen [here](https://youtu.be/OncJcgg6dec)

## The software tools used:

* Matlab 8.5.0.197613 (R2015a)
* V-REP PRO EDU version 3.5.0

## The simulation robot used:

* ePuck

## To run the simulation:

1. Start the V-REP simulator and open the simulation scene (epuck-dyn-obs-tar.ttt) provided in /src-vrep
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

<img src="/pics/dynamics-plot.png" alt="alt text" width="370">

4. If necessary, change the parameters of the dynamics and re-run the simulation

## Author

* Farid Oubbati
* Date: 12-May-2017
* Copyright (c) 2017

## License

This project is licensed under the MIT License - see the [LICENSE.txt](LICENSE.txt) file for more details
