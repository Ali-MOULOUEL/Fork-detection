import sensor_msgs.point_cloud2 as pc_utils
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import rospy
import numpy as np
from rosbag import Bag
import matplotlib.pyplot as plt
from PointcloudPublisher import publish_pointcloud, rotation, translation
from tf import TransformListener


def singlePlot(npz, step):
    fromfile = np.load('/home/project/formation_antonin/bagFile/'+ npz, allow_pickle=True)
    bins = np.arange(0,max(fromfile['data1']), step)
    distanceForkDetected = fromfile['data1'][fromfile['data2']]
    distanceForkNotDetected = fromfile['data1'][np.invert(fromfile['data2'])]
    serie1,edges1 = np.histogram(distanceForkDetected, bins=bins)
    serie2,edges2 = np.histogram(distanceForkNotDetected, bins=bins)
    RationOfDetection = (serie1/(serie1+serie2))*100
    serie3,edges3 = np.histogram(RationOfDetection, bins=bins)
    xCoordinate = (edges3[:-1] + edges3[1:])/2
    plt.bar(xCoordinate, RationOfDetection, width=step, edgecolor='black', color="green")
    metadata = fromfile["metadata"].tolist()
    plt.title("Histogram of detection ratio for foklift_y : " + str(metadata["forklift_y"])+ " and fork_elevation : " + str(metadata["fork_elevation"]))
    plt.xlabel("Distance (m)")
    plt.ylabel("Detection ratio (%)")
    plt.show()

singlePlot('2021-06-02.08-51-36.1.npz',1)
singlePlot('2021-06-02.08-57-11.3.npz',1)


