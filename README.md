# udacity_lidar
Repository for hosting the lidar project

Build instructions
====================
mkdir build
cd build
cmake ..
make


Sensors information
====================
I haven't uploaded the pcd folders with the sensors data, so this need to be copied manually under src/sensors.

Running the code without parameters
====================================
cd build
./environment
This will launch the project using data_1 with my predefined parameters that I considered worked the best

Running the code with a parameters file
=======================================
This is the most interesting option, since it allows to dynamically change the most of the parameters used in the code and see inmediately the result. As an example, I have included in the root directory 2 parameter sample files:
params_1.txt : LiDAR project using data set 1
params_2.txt : Extra challenge to follow the cyclist in front of the car

To run the program using the parameter files, just:
./environment ../params_1.txt
or
./environment ../params_2.txt

If you keep the parameter file open in an editor, you can change parameters live. The changes will be applied when you save to disk the changes.

Have fun!


