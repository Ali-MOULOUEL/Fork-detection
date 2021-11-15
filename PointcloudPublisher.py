
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2 as pc2
from ros_numpy.point_cloud2 import fields_to_dtype
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from scipy.spatial.transform import Rotation

def publish_pointcloud(publisher, pointcloud, msg=PointCloud2, convention={'x':0, 'y':1, 'z':2, 'intensity':3}):
    columns_dtypes = create_dtypes(msg, convention.keys())
    columns_fields = create_fields(convention.keys())
    # msg.header.frame_id="vehicle"
    output_msg = pc2.create_cloud(msg.header, 
                                  columns_fields,
                                  array_to_pointcloud(pointcloud, columns_dtypes))
    publisher.publish(output_msg)

def create_dtypes(msg, field_ids):
    return fields_to_dtype(create_fields(field_ids), msg.point_step)

def create_fields(colums_id=[]):
    fields = []
    counter = 0
    if 'x' in colums_id:
        fields.append(PointField('x', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'y' in colums_id:
        fields.append(PointField('y', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'z' in colums_id:
        fields.append(PointField('z', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'intensity' in colums_id:
        fields.append(PointField('intensity', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'distance' in colums_id:
        fields.append(PointField('distance', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'heading' in colums_id:
        fields.append(PointField('heading', counter, PointField.FLOAT32, 1))
        counter += 4
    if 'ring' in colums_id:
        fields.append(PointField('ring', counter, PointField.UINT8, 1))
        counter += 1
    if 'echoId' in colums_id:
        fields.append(PointField('echoId', counter, PointField.UINT8, 1))
        counter += 1
    return(fields)

def array_to_pointcloud(array, dtype):
    array = np.array(array)
    return np.array([tuple(row) for row in array], dtype=dtype[:array.shape[-1]]) 

def rotation(pointcloud, roll=0, pitch=0, yaw=0):
    rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
    pointcloud[:, :3] = rot.apply(pointcloud[:, :3])
    return pointcloud

def translation(pointcloud, x, y, z):
    pointcloud[:, :3] = pointcloud[:, :3] + [x, y, z]
    return pointcloud