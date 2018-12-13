# PointCloudAnnotationTool
A tool written in C++ to help annotate point clouds. The idea is to have a desktop tool that can help annotate/segment point clouds in order to build a dataset for 3D computer vision problems. 

This is a first draft version of the tool, but it works (sort of). 

Demo of the tool: https://www.youtube.com/watch?v=lmQCoYmulUo

## Build and Installation
Most of the tool is written in C++, but we a part of workflow involves training a neural network, which is still in python. So, currently build/install workflow is little hack-y. But, I hope to engineer the entire tool more elegantly, sometime in the future (collaborators welcome!).

For now, here is what you need to do:

### Pre-requisites
1. You need to install PCL and OpenCV on your machine (both of these excellent libraries have their own platform specific installtion guides, so I would recommend checking those out.)

2. Create a virtual environment and install Tensorflow on it. 

### Building the C++ project
Assuming you are in the root of this folder, run the following commands:

```
mkdir build
cd build
cmake ../prj/
make
```


If everything went fine so far, a build should be created by the name: ```point_cloud_annotation```. You can execute by the following command:

```
./point_cloud_annotation
```
