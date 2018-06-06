# sr_object_symmetry

This tool returns yaml(s) files with the corresponding rotational and reflectional symmetries of the corresponding CAD (.ply,.obj,.stl) models. The package accepts as input a single file or a folder path `objects_path`. The tool will save the yaml files in the defined directory `yaml_path`. 

The parameters are available in the launch file located:[sr_object_symmetry.launch](launch/sr_object_symmetry.launch)  that includes a description of each one of them. 

You can run the launch file of the tool with default arguments with: `roslaunch sr_object_symmetry sr_object_symmetry.launch` which will look for *.ply *.obj *.stl files in the default folder `sample_objects` and save the result yaml files in  project's `yalm` folder.

This package is based on [symseg](https://github.com/aecins/symseg) by Aleksandrs Ecins 