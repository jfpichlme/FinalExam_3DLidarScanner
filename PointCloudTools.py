import serial
import numpy as np
from sklearn.cluster import DBSCAN


import vispy
import vispy.scene
from vispy.color import ColorArray
import matplotlib.pyplot as plt


from vispy import app, visuals


import warnings




class ScanObject:
    """ ScanObject allows to create an Object which contains 3D Point Cloud Data and methods to process the data points
    Input params:


                 name: Gives the respective scan object a name

                 Resolution: Number of measurements for one Axis (Currently only measurements which have the same
                             Resolution on the x and y axis can be processed)
    """

    def __init__(self, name, Resolution):

        # Basic Parameters to define the Scan
        self.name = name  # Name of the Scan
        self.Resolution = Resolution  # Resolution defines the
        self.Length = []  # Vector that stores the Length measurements


        self.AngleMap = ScanObject.createAngleMap(self) # Containing the angles of the Scan depending on the Resolution

        # Processed coordinate data of the scan
        self.datax = np.array([])  # X coordinates of Scan
        self.datay = np.array([])  # Y coordinates of Scan
        self.dataz = np.array([])  # Z coordinates of Scan
        self.labels = np.array([])  # Labels after classification

    def createAngleMap(self):
        """ Returns an Array containing all the Angle values for the measurement points depending on the resolution.

            This Method assumes that the Liar scanner is initially positioned pointing 90 degrees upwards. It is then
            rotated by an angle of -180° (considered as the z plane) performing one measurement at each step.
            Having reached -90 degrees, it s then rotated by one step forward in the xy plane. the last measurement is
            assumed to happen when the sensor was rotated by 180°."""

        print("Creating Angle Map")

        angleStep = 180/self.Resolution  # Number of Steps done per 180 degrees
        xyAngle = 0  # Initial angle value for the xy plane
        zangle = 90  # Initial angle value for the z plane (90 = pointing upwards at the beginning)
        AngleList = []  # Array for the Angles
        flag = 1  # Flag to know if sensor is rotating upwards or downwards
        # cnt = 0

        # Loop for xy angle Values
        for xy in range(0, self.Resolution):
            xyAngle += angleStep  # calculating the xy Angle
            flag = flag * -1  # Resetting flag

            # Loop for calculating the z angle values
            for z in range(0, self.Resolution):
                if flag == -1:
                    zangle -= angleStep
                else:
                    zangle += angleStep

                angles = [xyAngle, zangle]  # Angle values for each measurement

                AngleList.append(angles)  # Appending angle values to array

        return AngleList

    def fillRandomData(self):
        """Test function to create random data and test new methods"""

        # Fill up Length array with values that are normally distributed
        for i in range(0, self.Resolution**2):
            self.Length.append(np.random.normal(6, 1))

    def loadPointCloudData(self, Filename):
        """Method that allows to load already created Point cloud data and save the coordinates in the respective arrays

            Filename: Name of the file that contains the Point Cloud data (.txt, os)
        """

        f = open(Filename, "rt")  # Open File

        for line in f:
            x = line.splitlines()  # Store one line in variable x

            for coord in x:
                coord.split()

                self.datax = np.append(self.datax, float(coord.split(',')[0]))  # x coordinate converted and stored
                self.datay = np.append(self.datay, float(coord.split(',')[1]))  # y coordinate conv. and stor.
                self.dataz = np.append(self.dataz, float(coord.split(',')[2]))  # z coordinate converted and stored

    def loadLidarScanValues(self, Filename):
        """Method that allows to load distance data. The number of values needs to be equal to the resolution squared

                Filename: Name of the file containing the length data"""

        f = open(Filename, "rt")  # Open file

        for line in f:
            x = line.splitlines()  # Split lines

            for y in x:
                self.Length.append(float(y))  # Store in Length array

        if( len(self.Length) != self.Resolution**2):
            warnings.warn("The number of length values should match the squared Resolution")




    def saveScan(self):
        """Method that allows to save the Length data and the coordinates in the Files "Scan1" and "Leng1" """

        DataFile = open("Scan1.txt", 'w')  # Open the files for the Length and the Coordinates
        LengFile = open("Leng1.txt", 'w')

        # Loop for saving the length data and coordinates
        for i in range(0, len(self.datax)):

            DataFile.write(str(self.datax[i]) + "," + str(self.datay[i]) + "," + str(self.dataz[i]))
            DataFile.write("\n")  # Write the Coordinates in a file

            LengFile.write(str(self.Length[i]))
            LengFile.write("\n")  # Write the Length in a file

        # Closing the files
        DataFile.close()
        LengFile.close()

    def __checkInput(self, input):
        """Helper function to check the input coming from the LIDAR scanner """
        # Try to convert the input to float (Length) if possible return true if not return false
        try:
            float(input)
            return True
        except ValueError:
            return False

    def lidarSensorRead(self):
        """Method that allows to receive Length data coming from a LIDAR sensor through serial port 'com3' """

        # Defining the Serial Port where the distance Data is arriving and with the data rate 115200 bits per second
        arduinoSerialData = serial.Serial('com3', 115200)

        ctn = 0  # Counter for the number of measurements arriving at the comport

        # while the counter is smaller then the number of expected point, read data from comport
        while ctn < self.Resolution**2:

            # If there is data waiting to be received, start reading data
            if arduinoSerialData.inWaiting() > 0:

                mydata = arduinoSerialData.readline()  # read data from serial port as string 'utf-8'

                # If the received data is a float
                if(ScanObject.__checkInput(self,mydata.decode("utf-8"))) == True:
                    t = float(mydata.decode("utf-8"))
                    print(t) # Print measurement data
                    if t > 65000:  # Filter out error value which is based on power supply factors
                        print("Error value stored as 0: ", t)
                        ctn += 1  # Even though it is an error, it has to be counted as a measurement
                        self.Length.append(t)  # Append to array
                    else:
                        ctn += 1  # If the value is an ordinary measurement store it
                        print("Progress: ", (ctn*100)/(self.Resolution**2), "%" )
                        self.Length.append(t)

                # Print messages from LIDAR sensor
                elif (ScanObject.__checkInput(self, mydata.decode("utf-8")) == False):
                    print("Following Message received from Sensor: ", mydata.decode("utf-8"))

    def _transformData(self):
        """Function to transform the Length data into xyz coordinates depending on the angle map """
        i = 0
        for dist in self.Length:
            self.datax = np.append(self.datax, self.Length[i] * np.cos(self.AngleMap[i][0]*2*np.pi/360)
                                   * np.cos(self.AngleMap[i][1]*2*np.pi/360))
            self.datay = np.append(self.datay, self.Length[i] * np.sin(self.AngleMap[i][0]*2*np.pi/360)
                                   * np.cos(self.AngleMap[i][1]*2*np.pi/360))
            self.dataz = np.append(self.dataz, self.Length[i] * np.sin(self.AngleMap[i][1]*2*np.pi/360))
            i += 1

    def mergescanobjects(self):
        print("To be implimented")

    def __str__(self):
        """Print most important information about the Point Cloud Data"""
        return str("Point Cloud " + self.name + " contains: " + str(len(self.Length)) +
                   " Points given by a resolution of " + str(self.Resolution) + ". The object can be separated into "
                   + str(len(set(self.labels)) - (1 if -1 in self.labels else 0)) + " clusters")

    def clustering(self):
        """Method that allows to create clusters in the Point cloud data based on the DBSCAN Algorithm"""

        # Create NpArray containing the point cloud of form (Resolution**2, 3)
        PointCloudData = np.vstack([self.datax, self.datay, self.dataz]).transpose()

        print("Finished normalization starting clustering")

        # Start DBSCAN algorithm with predefined hyper parameters
        clustering = DBSCAN(eps=7, min_samples= 8, leaf_size= 80).fit(PointCloudData)

        print("Finished Clustering")

        core_samples_mask = np.zeros_like(clustering.labels_, dtype=bool)
        core_samples_mask[clustering.core_sample_indices_] = True
        # Get the number of labels for each data point and store them in an np array
        self.labels = np.append(self.labels, clustering.labels_)

        # Get the total number of labels, label -1 = noise
        n_clusters_ = len(set(self.labels))- (1 if -1 in self.labels else 0)
        # Get the number of points which are defined to be noise
        n_noise_ = list(self.labels).count(-1)

        print("We derived a total number of ", n_clusters_, " clusters")
        print("Noise level: ", n_noise_/len(self.labels))

    def __getcolormap(self, unique_labels):
        """Helper function to create Dictionary for associating colors with labels"""

        colormap = {}  # Dictionary for color with key = label and value = RGB color

        # Get different colors for each label
        col = [plt.cm.Spectral(each) for each in np.linspace(0, 1, len(unique_labels))]

        i = 0
        for label in unique_labels:

            if label == -1:
                i += 1
                colormap[-1] = (1, 1, 1, 1)  # If point = noise, define color white
            else:
                i += 1
                colormap[label] = (col[i])  # If point is associated with label define color

        return colormap

    def plotpointclouddata(self):
        """Method that allows to Visualize the Point Cloud data and explore the landscape in fly mode using Vispy

            Keys to Navigating in Landscape:
                                                -> moving right
                                                <- moving left
                                                up moving forward
                                                down moving backwards
                                                f moving upward in z dimension
                                                c moving downward in z dimension

                                                j changing view towards left
                                                k changing view towards right
                                                i changing view upwards
                                                k changing view downwards

        """

        # Creating a visual node which allows to display points it inherits from the class Visuals.MarkersVisual
        # allowing to disply data points
        Scatter3D = vispy.scene.visuals.create_visual_node(vispy.visuals.MarkersVisual)

        # Create a SceneCanvas to display the scene
        canvas = vispy.scene.SceneCanvas(keys='interactive', show=True)

        # Create a new ViewBox
        view = canvas.central_widget.add_view()

        # Define settings for view
        view.camera = 'fly'  # 'Explore mode: 'fly' = flightsimulator, 'turntable' rotating around a fixed point
        view.camera.fov = 80  # Field of view
        view.camera.distance = 7  # Initial distance from origin
        view.camera.scale_factor = 10  # "thruster speed"  for mode fly

        colors = np.array([1, 1, 1, 1])  # Creating array containing the colors for the labels in size (4, 1)

        pos = np.c_[self.datax, self.datay, self.dataz]  # np array containing the data points

        if len(self.labels) == 0:  # Check if labels have already been created, if not plot every point as white
            print("For showing clusters, first apply the data clustering method")
            colors = ColorArray(color = (1, 1, 1, 1))

        else:
            unique_labels = set(self.labels)  # Get the number of unique labels with the set function
            colormap = ScanObject.__getcolormap(self, unique_labels)  # get the colormap from the helper function



            for label in self.labels:
                colors = np.vstack((colors, colormap[label]))  # associate a color to each point using the colormap

            colors = np.delete(colors, 0, axis=0)  # delete the first dummy entry defining the form of the array
            colors = ColorArray(colors)  # color array creating different color formats such as RGB



        #  Define plot
        p1 = Scatter3D(parent=view.scene)

        # Define the set of Open GL state params to use when drawing
        p1.set_gl_state('translucent', blend=True, depth_test=True)

        # Set the data and define options for it
        p1.set_data(pos,
                    face_color=colors,
                    size = 3.5,
                    edge_width=0.1,
                    edge_color=None
                    )

        # Show the XYZ Axis
        xis = vispy.scene.visuals.XYZAxis(parent=view.scene)

        # Start plotting
        vispy.app.run()


# Functions that use the Actual 3D Hardware implementation

#FirstScan = ScanObject("First_Try", 178)
#FirstScan.lidarSensorRead()
#FirstScan.fillRandomData()
#FirstScan._transformData()
#FirstScan.saveScan()
#FirstScan.clustering()
#FirstScan.plotpointclouddata()





# Function that loads Length Data

TryLoadScan = ScanObject("FirstLoadTry", 178)
TryLoadScan.loadLidarScanValues("Leng1.txt")
TryLoadScan._transformData()
TryLoadScan.clustering()
TryLoadScan.plotpointclouddata()
print(TryLoadScan)


# Function that loads stored xyz data

#TryLoadScan = ScanObject("FirstLoadTry", 178)
#TryLoadScan.loadPointCloudData("Scan1.txt")
#TryLoadScan.clustering()
#TryLoadScan.plotpointclouddata()
print(TryLoadScan)



