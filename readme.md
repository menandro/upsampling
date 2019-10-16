# Real-Time Dense Depth Estimation Using Semantically-Guided LIDAR Data Propagation and Motion Stereo

MATLAB implementation of our paper submission for RAL/IROS2019.

Atsuki Hirata, Ryoichi Ishikawa, Menandro Roxas, Takeshi Oishi. "[Real-Time Dense Depth Estimation Using Semantically-Guided LIDAR Data Propagation and Motion Stereo](https://ieeexplore.ieee.org/document/8756107)", IEEE Robotics and Automation Letters, Volume 4, Issue 4, 2019.

## Requirements
1. Tested on MATLAB R2019a

## Dataset
Our dataset can be accessed from: [dataset](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/data_iros2019_open.zip). There are 56 image pairs with ground truth depth, simulated LIDAR data, semantic segmentation results from [ICNET](https://github.com/hellochick/ICNet-tensorflow), [motion stereo](https://github.com/menandro/Reconflow) and additional [flownet2](https://github.com/lmb-freiburg/flownet2) optical flow for reference. We also provided manually segmented images.

Folder Structure:
* calib - intrinsic camera matrix K, and transformation R, t between first->second image frames in OpenCV format .xml 
* image_02/data - first RGB image frame
* image_03/data - second RGB image frame
* motionstereo - depth from motion stereo in 16-bit .png
* proj_depth/groundtrutn - contains groundtruth depth for both image_02 and image_03 
* proj_depth/velodyne_raw - contains simulated LIDAR data for image_02
* semantic_icnet - semantic segmentation from ICNET
* semantic_manual - manual semantic segmentation (subset only)

Miscellaneous:
* flownet2, flownet2_CSS - optical flow (for reference only)
* result_ours - our results for reference


RGB
![RGB](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/image.png) | ![Semantic segmentation](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/semantic.png)
Semantic segmentation
![Semantic segmentation](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/semantic.png)
Motion stereo
![Motion stereo](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/motionstereo.png)
LIDAR
![LIDAR](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/lidar.png)
Ground truth depth
![Ground truth depth](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/gtdepth.png)
Result-propagation
![Result-propagation](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/propagation.png)
Result-optimization
![Result-optimization](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/optimization.png)

## Building Instructions
Sample program is in upsampling_main.m.

## Sample Result on KITTI
![Video](http://b2.cvl.iis.u-tokyo.ac.jp/~roxas/iros2019resultgithub.mp4)

## To Do
1. Release C++/CUDA codes.

## License
This project is licensed under the MIT license

## Author
Atsuki Hirata and Menandro Roxas, 3D Vision Laboratory (Oishi Laboratory), Institute of Industrial Science, The University of Tokyo


