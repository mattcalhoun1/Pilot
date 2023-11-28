# Pilot
This application serves as a 'Pilot' for supported vehicles (see Vehicles repository). It is intended to run on a platform such as Raspberry Pi or Nvidia Jetson Nano.

This is part of a capstone project I am working on for may Master's degree. The project is centered around navigation based entirely on vision (no gps).

Essentially, if you create a json map of an area, declaring x,y coordinates where certain objects are expected to be visible, this system is able
to monitor dual picameras for objects mentioned on the 'map', and use their known position to triangulate its own coordinates and heading (relative to the map).

This is highly experimental, and as you will find, there are largely untested and unpolished portions of code. This is not intended to be used in any sort of production/live environment at the moment. It is very much a work in progress.
