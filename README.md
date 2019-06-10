# Design of a positioning and a mission management system for builder drones

## [UCLouvain](https://uclouvain.be/fr/index.html) - [EPL](https://uclouvain.be/en/faculties/epl) - Master Thesis

## Credit
Authors : [Amalaberque Julien](https://github.com/jamalaberque), [van der Beek Marie-Marie](https://github.com/mvanderbeek123) & [Nicolas Vanvyve](https://github.com/NVanvyve)

Supervisors : [Pierre Latteur](https://uclouvain.be/fr/repertoires/pierre.latteur) & [Ramin Sadre](https://uclouvain.be/fr/repertoires/ramin.sadre)

Assistant : [Sebastien Goessens](https://uclouvain.be/fr/repertoires/sebastien.goessens)

## Abstract
This master's thesis is part of the research project led by Pr. Pierre Latteur and his team at the Civil Engineering Department of UCLouvain, on the use of drones for building real-life structures to robotize construction processes.

In previous years, drone compatible construction elements and different positioning systems for the drone were developed using multiple technologies: ultra-wideband, total station and computer vision.
The next step was to integrate those systems together in a single more robust and reliable positioning system that could send its information to the flight controller unit.

Additionally, a guidance system had to be developed to allow the planning of flight missions and their correct execution by the drone.

<!-- In this report we will explain the development of these two steps and show experimental results of flight missions. -->

## File organization

###### ROStotalSender
In this folder are the files that must be on the computer that is connected to the total station.
To launch the tracking script with the server that sends the positions (only works on windows):
 `ROStotalSender.exe comX 57600`

###### src
This contains the code of the different systems.

###### launch
This folder contains the launch files for the different systems.

## Usage
All the information can be found in Appendix B of our [report](report/master_thesis_report.pdf) and in [README.md](launch/README.md)
