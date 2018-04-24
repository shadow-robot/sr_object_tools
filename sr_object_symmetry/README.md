# sr_object_symmetry

This package allows you do extract the rotational and reflectional symmetries of an object. It supports .ply objects as input and outputs an array of symmetries (reflectional and rotational) 
Have a look at the example for its usage. 

## Dependencies ##
- [PCL](https://github.com/PointCloudLibrary/pcl)
- [OctoMap](https://github.com/OctoMap/octomap) (with dynamicEDT3D)

Tested in Ubuntu 14.04 and 16.04 with PCL 1.8.0 and Octomap 1.8.1


## Building ##

To checkout and build *sr_object_symmetry* in the `build` directory, execute the following in a terminal:

```
mkdir build
cd build
cmake ..
make
```

## Examples ##
The `examples` directory provides an example on how to extract symmetries from the objects located in the sample_objects folder

To extract and display the symmetries execute the following from the `bin` directory:
```
./rotational_reflectional box.ply (or any other .ply object in that folder)
```
