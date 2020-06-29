# FinalExam_3DLidarScanner
This repository includes a python code for visualizing and analyzing Point Cloud Data. The example point cloud data is derived using a TFMini and an Arduino mega. The code controlling the hardware is included as well but irrelevant for the Final Exam. It is added for users who own a Tfmini scanner attached to an XYZ manipulator controlled by two stepper motors. 

## Getting Started 
The PointCloudTools code is highly dependent on the library Vispy, which is designed for interactive scientific visualization. It allows to efficiently visualize a large number of points. Following libraries are therefore required for using PointCloudTools: 

```
pip install vispy
```

```
pip install PySide2
```

For receiving data from a Lidar Scanner, the library pyserial is used

```
pip install pyserial
```


For performing segmentation on the data, the DBSCAN algorithm is used. It is included in the library sci-kit-learn: 

```
pip install scikit-learn
```


## Running the tests 
The test codes are using the data of two lidar scans performed for two different rooms. The results (XYZ coordinates) are saved in the files "PointCloudData_BedRoom.txt" and "PointCloudData_LivingRoom.txt". For the sake of completeness, the raw length data is included as well, allowing to test the coordinate transformation for deriving the XYZ coordinates. For both scans, a resolution of 178x178 is chosen (= 31,684 measurements).
By running Test_LidarScanLoad.py, the Point Cloud for the Bed Room is visualized at first. The section "Vispy Flight Emulator" explains how to explore the resulting 3D plots. The second plot illustrates the derived clusters after applying the DBSCAN algorithm. Finally, the scans of the living room are shown.

Test_LidarScan.py allows to test a 3D Hardware Scanner attached to Com Port 3. As already mentioned, the test files have been created using a TFmini with a baud rate of 115200. However, to allow a quicker scan, the resolution in Test_LidarScan.py is set to 89x89 (= 7921 measurements). 

Test_CombineScans.py is testing the method that allows to combine different scans. The plot is showing the two previously mentioned scans combined. 


## Remark on the Results 
The TFmini is a comparably cheap LIDAR scanner. It has a maximum range of approx. 12 m and a minimum range of 30 cm. The XYZ manipulator which it is attached to is not very precise allowing minimal steps of approx 1.02 degrees. Nevertheless, the plots show the rooms with a high degree of detail. This holds especially true for the bedroom data. Since the DBSCAN algorithm depends on the density of the points, it allows performing a quite accurate segmentation on the bedroom data. However, regarding the living room data, the DBSCAN algorithm is labeling many of the data points as noise. This can be caused by the fact that this room is comparably large resulting in generally lower point density. However, the DBSCAN algorithm was still able to put objects such as the guitar or parts of the sofa into one cluster. 


## Vispy Flight Emulator
Keys to navigating in the 3D landscape:

                                                -> moving right
                                                <- moving left
                                                up moving forward
                                                down moving backward
                                                F moving upward in the z dimension
                                                C moving downward in the z dimension

                                                J changing view towards left
                                                L changing view towards the right
                                                I changing view upwards
                                                k changing view downwards
