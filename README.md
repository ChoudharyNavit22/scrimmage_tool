# Scrimmage Simulation

The simulation is based on gtri/scrimmage. so inorder to run this you would have to first clone, build and run the scrimmage repo.

The scrimmage repo can be found on the [link](https://github.com/gtri/scrimmage). Currently the scrimmage best works on an ubuntu machine so it is advised to please try and use an ubuntu environment for the project.

## Build

    $ mkdir build
    $ cd build
    $ cmake ..
    $ make
    $ source ~/.scrimmage/setup.bash

## Run The Project

    $ cd missions
    $ scrimmage test.xml
    
