# Week 6-7 Instructions
- [Introduction](#Introduction)
    - [An index of provided scripts and resources](#An-index-of-provided-scripts-and-resources)
    - [Neural network with PyTorch](#Neural-network-with-PyTorch)
    - [Training and improving your neural network](#Training-and-improving-your-neural-network)
- [Objectives](#Objectives)
- [Marking schemes](#Marking-schemes)
- [Getting-started](#Getting-started)
    - [Data collection (Week 6)](#Data-collection-Week-6)
    - [Training and improving the neural network (Week 7)](#Training-and-improving-the-neural-network-Week-7)

## Introduction
### An index of provided scripts and resources
Helper scripts (no changes required)
1. [dataCollection.py](dataCollection.py): this script can be used together with your keyboard teleoperation script to create additional trianing data
2. [dataSegmentation.py](dataSegmentation.py): this script segments collected data into training, evaluation, and testing sets
3. [dataset_provided.zip](dataset_provided.zip): this is a provided dataset of 600 images
4. [nn_config.py](nn_config.py): this script passes on parameters to the network and saves the trained network
5. [yml_utils.py](yml_utils.py): this script loads the parameters of saved network
6. [nn_train.py](nn_train.py): this script is used for training a neural network with the training and evaluation sets
7. [nn_test.py](nn_test.py): this script tests a trained network on the unseen testing set

Scripts that may be modified to improve performance
1. [nn_config.yml](nn_config.yml): this script defines paprameters that are key to the network's performance. It's the main script that you will be tweaking other than collecting more data to improve your network's performance
2. [baseline_net.py](baseline_net.py): this script implements the network structure. You may change it if you would like to use a different network (not required)
3. [nn_detect.py](nn_detect.py): this script applies a trained network to a given image ([test.png](test.png)) and visualised the detection result. It will be reused in the integrated system for detecting targets and can be modified in M5

### Neural network with PyTorch
Install [PyTorch](https://pytorch.org/) (select "Pip" as Package and "None" for CUDA) and other dependencies:

```
python3 -m pip install pandas 
python3 -m pip install torch==1.6.0+cpu torchvision==0.7.0+cpu -f https://download.pytorch.org/whl/torch_stable.html pip install torch==1.6.0+cpu torchvision==0.7.0+cpu -f https://download.pytorch.org/whl/torch_stable.html
python3 -m pip install -U PyYAML
```

The neural network model implemented in [baseline_net.py](baseline_net.py) is the [AlexNet](https://pytorch.org/hub/pytorch_vision_alexnet/). PyTorch also provides a number of [other image classification models](https://pytorch.org/docs/stable/torchvision/models.html), which you may explore further (not required).

### Training and improving your neural network
1. Data collection
    - Take photos from different angles with different backgrounds
2. Segment collected data into training, evaluation, and testing sets
    - Tune your parameters on the evaluation set
    - Use the testing set to check the robustness of your neural network to avoid overfitting
3. Parameter tuning: see [nn_config.yml](nn_config.yml)
4. Local VM should be sufficient for training this simple neural network with this small dataset, no GPU is needed

## Objectives
1. [Week 6](#Data-collection-Week-6): Collect data to train a neural network that performs a 3-way classification of images: sheep, coke, neither
2. [Week 7](#-Training-and-improving-the-neural-network-Week-7): Tune the parameters to improve your neural network

## Marking schemes
Classification accuracy: 
- Please submit your trained neural network by submitting the "net_weights" folder. If you have made changes to any of the python / yaml scripts please submit them as well.
- Your trained model will be applied to a marking test set of 60 unseen images to predict their labels. 
- If you have experimented with changing the neural network structure or using a differen neural network, you will be given a maximum of 10pts of exploration points. You will need to submit the custom neural network implementation, its accompanying training / testing / configuration functions, and a screenshot of the training printouts to demonstrate your exploration.
- Your score (max 100pt) = the accuracy achieved by your trained network on the marking test set + exploration points. For example, if your best trained network achieved an accuracy of 85% on the test set and you have experimented with using VGG instead of AlexNet, then you will get 95pts (85+10) for M3.

## Getting-started
**Note: make sure you have saved your previous development elsewhere before pulling the repo.**

### Data collection (Week 6)
1. Provided data
    - 600 labeled images are provided (200 sheep, 200 coke, 200 neither) in [dataset_provided.zip](dataset_provided.zip). Extract it into your directory and create another folder in your directory named "dataset_segmented". 
    - Run [dataSegmentation.py](dataSegmentation.py) to segment the provided images into training, evaluation, and testing sets with balanced categories. The segmented dataset is inside the "dataset_segmented" folder. The ratio of segmentation is (training, evaluation, testing) = (60%, 20%, 20%). 
2. Collecting more data
    - Launch the cardboard arena as specified in [Week03-05's instructions](https://github.com/tianleimin/ECE4078_Lab/tree/master/Week03-05#Launch-the-cardboard-arena-world-week-3).
    - If you would like to collect additional data or use your own dataset, please use [dataCollection.py](dataCollection.py). Note: it requires [penguinPiC.py](https://github.com/tianleimin/ECE4078_Lab/blob/master/Week01-02/penguinPiC.py) to run. 
    - You can either drive the robot around with your keyboard teleoperation function while running [dataCollection.py](dataCollection.py) in another terminal to collect data (make sure the number keys and ENTER key are NOT used in your keyboard teleoperation function), or drag and drop the robot / figurines inside Gazebo.
    - Make sure that you are taking photos from different angles / distances to improve the robustness of your model.
    - After collecting more data, you will need to rerun [dataSegmentation.py](dataSegmentation.py) to segment the grown dataset into training, evaluation, and testing sets.

### Training and improving the neural network (Week 7)
1. Create a folder in your directory named "net_weights". Run [nn_train.py](nn_train.py) to train a baseline neural network. You can monitor the training process and its accuracy on the evaluation set in the terminal outputs. The trained neural network will be saved in the "net_weights" folder.
2. After the training is done, run [nn_detect.py](nn_detect.py), which helps you visualize how the neural network detects sheep / coke / neither in different sements of a testing image [test.png](test.png).

![Heat map of an example neural network detection outputs](https://github.com/tianleimin/ECE4078_Lab/blob/master/pics/test_nn_detect_output.png?raw=true "Heat map of an example neural network detection outputs")

3. Make changes to [nn_config.yml](nn_config.yml) and retrain your model until you are happy with its performance. You may also [change the neural network model](#Neural-network-with-PyTorch) implemented in [baseline_net.py](baseline_net.py).
4. Once you are satisfied with your network's performance on the evaluation set, test the trained model on the test set by running [nn_test.py](nn_test.py). This will give you an idea how well the trained model generalizes to unseen data.

