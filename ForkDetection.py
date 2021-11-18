import argparse
import os
import pandas as pd
import numpy as np
import sensor_msgs.point_cloud2 as pc_utils
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
import rospy
import numpy as npcel
from rosbag import Bag
import matplotlib.pyplot as plt
from PointcloudPublisher import publish_pointcloud, rotation, translation
from tf import TransformListener

def parse_args():                                   
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=str)
    parser.add_argument("scenarios", type=str)
    return parser.parse_args()

def msgToPointcloud(msg):
    """Read pointcloud as numpy array."""
    pointcloud = np.array(list(pc_utils.read_points(msg)))
    return pointcloud

def grandFilter(pc, z):
        selector = pc[:,2] > z
        return pc[selector]

class BoundingBox:
    """BoundingBox object to be applied to one or more pointclounds."""

    def __init__(self, center_point, height, width, length, id=None):
        self.center_point = center_point
        self.half_height = height / 2
        self.half_width = width / 2
        self.half_length = length / 2
        if id is not None:
            self.id = id
            self.vizualize()

    def filter(self, pointcloud):
        """Apply bounding box filter to a numpy-formatted pointclound."""
        box_locator = np.logical_and.reduce(
            # distance(y,c_y) < box_width/2
            (np.abs(pointcloud[:, 0] - self.center_point[0]) < self.half_length,
             # distance(x,c_x) < box_length/2
             np.abs(pointcloud[:, 1] - self.center_point[1]) < self.half_width,
             # distance(z,c_z) < box_height/2
             np.abs(pointcloud[:, 2] - self.center_point[2]) < self.half_height))
        return pointcloud[box_locator]

    def vizualize(self):
        """Publish a RViz-comaptible 'Box' marker matching the bounding box."""
        from visualization_msgs.msg import Marker
        topic = 'boundingBox_{}'.format(self.id)
        publisher = rospy.Publisher(topic, Marker, queue_size=1)
        # rospy.init_node('register')
        marker = Marker()
        marker.header.frame_id = FRAME_ID
        marker.type = marker.CUBE 
        marker.scale.x = 2 * self.half_length
        marker.scale.y = 2 * self.half_width
        marker.scale.z = 2 * self.half_height
        marker.color.a = 0.4
        marker.color.r = 0.5
        marker.color.g = 0.5
        marker.color.b = 0.5
        marker.pose.position.x = self.center_point[0]
        marker.pose.position.y = self.center_point[1]
        marker.pose.position.z = self.center_point[2]
        publisher.publish(marker)

if __name__ == "__main__":
    
    args = parse_args()
    print(args.bag)
    print(args.scenarios)
    BAG_FILE=args.bag
    data = pd.read_csv(args.scenarios)
    TOPICS=["/scan_vlp32_front"]
    FRAME_ID="vlp32_front"
    # Extract bag name : basename() give the file name from a full path
	#                    filename[:-4] remove the last 4 character ('.bag') 
    bag_name = os.path.basename(args.bag)[:-4]
    print(bag_name)
	# Read .csv file as a pandas DataFrame
    scenarios = pd.read_csv(args.scenarios, delimiter=";", index_col=0)
	
	# Extract the line where the bag_id column matches the bag name
	# Also, extract it as a dict 
    metadata = scenarios.query('bag_id == "{:s}"'.format(bag_name)).to_dict('records')[0]
    print(metadata)

    bag_data = Bag(BAG_FILE)

    rospy.init_node("ForkDetection", anonymous=True)
    listener = TransformListener()

    publisher_acquisition = rospy.Publisher("/ForkDetection/acquisition", PointCloud2, queue_size=1)
    publisher_transformed = rospy.Publisher("/ForkDetection/transformed", PointCloud2, queue_size=1)
    publisher_filtered = rospy.Publisher("/ForkDetection/filtered", PointCloud2, queue_size=1)
    publisher_alpha = rospy.Publisher("/ForkDetection/alpha", PointCloud2, queue_size=1)
    publisher_grandFilter = rospy.Publisher("/ForkDetection/grandFilter", PointCloud2, queue_size=1)
    publisher_forklift = rospy.Publisher("/ForkDetection/forklift", PointCloud2, queue_size=1)

    message_id = 0
    alpha = 0
    xCorridor = 0
    len = 150
    forkliftDistance = []
    detectionsOfFork = []
    forkDistance = []

    fly2cy = {0 : 0,                        #Dictionnary to choose the Y coordinate for BBox corriodor
              30 : 3, 
              -30 : -3, 
              150 : 5, 
              -150 : -5} 
    yCorridor = fly2cy[metadata["forklift_y"]]

    if(metadata["forklift_y"] > 0):
        boxAlphaYCoordinate = (yCorridor - 4)
    else :
        boxAlphaYCoordinate = (yCorridor + 4)
        
    boundingBoxAlpha = BoundingBox(center_point=(50,boxAlphaYCoordinate,0),
                                   height=2,
                                   width=0.5,
                                   length=100,
                                   id='BoundingBox1')
    
    boundingBoxCorridor = BoundingBox(center_point=(0,yCorridor,2),
                                                    height=6,
                                                    width=10,
                                                    length=500,
                                                    id='BoundingBox0')
    timestamp = None
    for topic, msg, t in bag_data.read_messages(topics=TOPICS):
        if not timestamp:
            timestamp = t.secs
            continue
        if (abs(timestamp - t.secs)) < (metadata["beginTime"]):
            continue

        print(message_id)
        message_id+=1

        boundingBoxEnclosingForklift = BoundingBox(center_point=(xCorridor,yCorridor,2),
                                                                 height=4,
                                                                 width=6.5,
                                                                 length=len,
                                                                 id='BoundingBox2')

        pointcloud = msgToPointcloud(msg)
        publisher_acquisition.publish(msg)                                              #Publish same pointcloud without processing

        pointcloud = translation(pointcloud=pointcloud,
                                 x=2.076,
                                 y=0.0,
                                 z=2.03)
        pointcloud = rotation(pointcloud=pointcloud,
                                 yaw=np.degrees(0.055),
                                 pitch=np.degrees(0.0524 + alpha),
                                 roll=np.degrees(0.0))

        publish_pointcloud(publisher=publisher_transformed,                             #Publish pointcloud translated and rotated
                            pointcloud=pointcloud[:,:4],
                            msg=msg)

        pointcloudCorridor = boundingBoxCorridor.filter(pointcloud)
        publish_pointcloud(publisher=publisher_filtered,
                           pointcloud=pointcloudCorridor[:,:4],
                           msg=msg)

        farAway = boundingBoxAlpha.filter(pointcloudCorridor)                           #Filter the faraway points with another BBcorridor1
        publish_pointcloud(publisher=publisher_alpha,
                           pointcloud=farAway[:,:4],
                           msg=msg)
                           
        grandfilter = grandFilter(pointcloudCorridor, 0.18)                             #Filter again the point located on the BBcorridor to eliminate point which has height lower than 20cm
        publish_pointcloud(publisher=publisher_grandFilter,
                           pointcloud=grandfilter[:,:4],
                           msg=msg)

        pointcloudEnclosingForklift = boundingBoxEnclosingForklift.filter(grandfilter)   #Filter the pointcloud to select only the point on the BBcorridor
        publish_pointcloud(publisher=publisher_forklift,
                           pointcloud=pointcloudEnclosingForklift[:,:4],
                           msg=msg)

        if(farAway.size > 0):                                                            #Estimate the farAwayPoint (x coordinate) to calibrate the pitch with an alpha angle
            xmax = np.max([farAway[:,0]])
            farAwayPoint = farAway[farAway[:,0] == xmax][0]
            alpha += np.arctan(farAwayPoint[2] / farAwayPoint[0])
 

        if(pointcloudEnclosingForklift.size > 0):                                        #Estimate the ymax of the fork to track it with BBcorridor
            zmax = np.max([pointcloudEnclosingForklift[:,2]])
            maxZpoint = pointcloudEnclosingForklift[pointcloudEnclosingForklift[:,2] == zmax][0]
            xCorridor = maxZpoint[0]
            yCorridor = maxZpoint[1]

            xmin = np.min([pointcloudEnclosingForklift[:,0]])
            minXpoint = pointcloudEnclosingForklift[pointcloudEnclosingForklift[:,0] == xmin][0]

            xmax = np.max([pointcloudEnclosingForklift[:,0]])
            maxXpoint = pointcloudEnclosingForklift[pointcloudEnclosingForklift[:,0] == xmax][0]

            ymin = np.min([pointcloudEnclosingForklift[:,1]])
            minYpoint = pointcloudEnclosingForklift[pointcloudEnclosingForklift[:,1] == ymin][0]

            ymax = np.max([pointcloudEnclosingForklift[:,1]])
            maxYpoint = pointcloudEnclosingForklift[pointcloudEnclosingForklift[:,1] == ymax][0]

            if(metadata["forklift_y"] == 0):
                len = 6
                longueur = abs(xmax-xmin)

            else:
                len = 3             
                longueur = abs(ymax-ymin)

            if longueur > 4:
                print("Longueur de la forklift: -----------------------------", longueur, "distance: ", xmin )
                forkliftDistance.append(xmin)
                forkliftDistance.append(True)
                forkDistance.append(xmin)

            else:
                forkliftDistance.append(xmin)
                forkliftDistance.append(False)
        print("Distance_detection: ", forkDistance)
        # .npz writing example
        output_file = args.bag[:-4]+".npz"  # will create the .npz file along the bag file
        print("data : ", forkDistance)
        np.savez_compressed(output_file, metadata=metadata, data=forkDistance, data1=forkliftDistance, data2=detectionsOfFork)
        """            
        # .npz reading example
        fromfile = np.load('/home/project/formation_antonin/forkDetection/2021-06-02.08-51-36.1.npz', allow_pickle=True)
        print("metadata from .npz file : ", fromfile['metadata'])
        print("data from .npz file : ", fromfile['data'])
        """
        boundingBoxCorridor.vizualize()
        boundingBoxEnclosingForklift.vizualize()
        boundingBoxAlpha.vizualize()

    plt.plot(forkliftDistance, forkliftDistance)
    plt.show()
