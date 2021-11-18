import sensor_msgs.point_cloud2 as pc_utils
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import rospy
import numpy as np
from rosbag import Bag
import matplotlib.pyplot as plt
from PointcloudPublisher import publish_pointcloud, rotation, translation
from tf import TransformListener

def singlePlot(npz, step, ax=None):
    if ax is None:
        fig, ax = plt.subplots()
    fromfile = np.load('/home/project/formation_antonin/bagFile/'+ npz, allow_pickle=True)
    bins = np.arange(0,max(fromfile['data1']), step)
    distanceForkDetected = fromfile['data1'][fromfile['data2']]
    distanceForkNotDetected = fromfile['data1'][np.invert(fromfile['data2'])]
    serie1,edges1 = np.histogram(distanceForkDetected, bins=bins)
    serie2,edges2 = np.histogram(distanceForkNotDetected, bins=bins)
    RationOfDetection = (serie1/(serie1+serie2))*100
    serie3,edges3 = np.histogram(RationOfDetection, bins=bins)
    xCoordinate = (edges3[:-1] + edges3[1:])/2
    ax.bar(xCoordinate, RationOfDetection, width=step, edgecolor='black', color="green")
    metadata = fromfile["metadata"].tolist()
    ax.set_title("Histogram of detection ratio for foklift_y : " + str(metadata["forklift_y"])+ ", and fork_elevation : " + str(metadata["fork_elevation"])+".")
    ax.set_xlabel("Distance (m)")
    ax.set_ylabel("Detection ratio (%)")

fig, axs = plt.subplots(2,sharex=True)
singlePlot('2021-06-02.08-54-42.2.npz',1, axs[0])
singlePlot('2021-06-02.09-01-11.4.npz',1, axs[1])
plt.show()


