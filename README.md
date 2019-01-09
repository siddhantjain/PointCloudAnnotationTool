# PointCloudAnnotationTool
A tool written in C++ to help annotate point clouds. The idea is to have a desktop tool that can help annotate/segment point clouds in order to build a dataset for 3D computer vision problems. 

This is a first draft version of the tool, but it works (sort of). 

Demo of the tool: https://www.youtube.com/watch?v=lmQCoYmulUo

## Build and Installation
Most of the tool is written in C++, but a part of workflow involves training a neural network, which is still in python. So, currently the build/install workflow is little hack-y. But, I hope to engineer the entire tool more elegantly, sometime in the future (collaborators welcome!).

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

If you encountered errors, please leave an issue on the repo. At this moment I don't know what all can go wrong in the installation workflow on various machines, with different versions of the dependencies, so will solve them as (and if) issues get reported.

## Using the tool
This is a command line tool right now. So, once you run the tool another terminal window will pop-up with hopefully the point cloud of your choice.

### How to indicate the point cloud to open?
```src/cfg_file.txt``` is a configuration file that you can use to set modifiable elements in the tool. Give the absolute (or relative to the executable) path to the point cloud file here. You can also indicate the number of classes you intend to annotate in this point cloud.

### So, the point cloud loads, how should I start annotating?
There are a bunch of keyboard shortcuts to get around things. Again, future engineering work is to build an entire UI around this, but for now, let's makedo with remembering a few key-action pairs. 

1. Mouse interactions:
To start off, you can use scroll to zoom-in/zoom-out. A middle click will help you pan the point cloud and a right click drag can be used for rotation. 

2. Basic Keyboard interactions
press `a` for getting into "annotation mode". Now you press `1` to `9` to select the class that you want to annotate. After this click on a point in the cloud to label it (and the system will automatically grow this selection). Once you are done selecting, press `i` to get out of annotation mode. Now you can press `w` to write this annotated point cloud to file. This point cloud will be written at `..\tmp\PointCloudFileName_labelled.txt`. Where "PointCloudFileName" is the name of your point cloud file. 

3. Advanced Keyboard interactions
In order to activate the finetuning workflow, you need to make sure you ran the tool from a virtual environment with the python dependencies installed (or have the python dependencies installed globally on your machine). Now, once you have given the intial annotation, press `i` to get out of the annotation mode, press `w` to write the labelled point cloud and then press `f` to start the fine-tuning workflow. If everything went fine, you will see, logs in your terminal for the fine-tuning. Once the fine-tuning is done, you can press `u` to load the predictions from the network. For simpler point clouds, this one step of finetuning should bring you pretty close to the actual solution, otherwise, you can try marking a few corrections (by pressing `a` and going into the annotation mode) and re-doing the fine-tuning workflow.

That's about all I wanted to document for now. Please consider the following resources for more information:

1. Project website: (https://mscvprojects.ri.cmu.edu/2018team10/)
2. Project report: (https://github.com/siddhantjain/PointCloudAnnotationTool/blob/master/FinalReport.pdf) 
3. Leaving an issue here
 
