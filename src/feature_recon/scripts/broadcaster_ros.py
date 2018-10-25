#!/usr/bin/python2
import time
import os
import sys
import ast
import struct
import math

from threading import Lock
import rospy
import rospkg
import message_filters
import sensor_msgs.point_cloud2 as pc2
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image, PointCloud2
from feature_recon.msg import Persons, Person, BodyPartElm

from tf_pose.estimator import TfPoseEstimator
from tf_pose.networks import model_wh, get_graph_path

def read_depth(width, height, data) :
    # read function
    if (height >= data.height) or (width >= data.width) :
        return -1
    data_out = pc2.read_points(data, field_names=None, skip_nans=False, uvs=[[width, height]])
    int_data = next(data_out)
    return int_data

def median(x, y, cloud) :
    items = []
    for i in range(-1,2):
        for j in range(-1, 2):
            items.append(read_depth(x-i,y-j,cloud)[2])
    items.sort()
    #print("Minimum: ", items[0], ", Maximum: ", items[len(items)-1])
    return items[len(items)//2]

def get_3d_distance(body_part_1, body_part_2):
    return math.sqrt(math.pow(body_part_2[0] - body_part_1[0], 2) + math.pow(body_part_2[1] - body_part_1[1], 2) + math.pow(body_part_2[2] - body_part_1[2], 2))

def humans_to_msg(humans, cloud):
    persons = Persons()

    for human in humans:
        person = Person()

        for k in human.body_parts:
            body_part = human.body_parts[k]
            body_part_msg = BodyPartElm()
            body_part_msg.part_id = body_part.part_idx
            depth_list = read_depth(int(body_part.x*640), int(body_part.y*480), cloud);
            body_part_msg.x = depth_list[0]
            body_part_msg.y = depth_list[1]
            body_part_msg.z = median(int(body_part.x*640), int(body_part.y*480), cloud)
            body_part_msg.confidence = body_part.score
            if not(math.isnan(body_part_msg.x) and math.isnan(body_part_msg.y) and math.isnan(body_part_msg.z)) :
                #print(body_part_msg.x, ", ", body_part_msg.y, ", ", body_part_msg.z);
                person.body_part.append(body_part_msg)
            print("Body parts = ", len(person.body_part))
        persons.persons.append(person)
    return persons


def callback_image(data, cloud):
    # et = time.time()
    try:
        cv_image = cv_bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr('[tf-pose-estimation] Converting Image Error. ' + str(e))
        return

    acquired = tf_lock.acquire(False)
    global humans
    if not acquired:
        return

    try:
        humans = pose_estimator.inference(cv_image, resize_to_default=True, upsample_size=resize_out_ratio)
    finally:
        tf_lock.release()

    image = pose_estimator.draw_humans(cv_image, humans)
    image_msg = cv_bridge.cv2_to_imgmsg(image, encoding='bgr8')
    pub_img.publish(image_msg)
    msg = humans_to_msg(humans, cloud)
    msg.image_w = data.width
    msg.image_h = data.height
    msg.header = data.header
    pub_pose.publish(msg)


if __name__ == '__main__':
    rospy.loginfo('initialization+')
    rospy.init_node('TfPoseEstimatorROS', anonymous=True, log_level=rospy.INFO)

    # parameters
    image_topic = rospy.get_param('~rgb', '')
    cloud_topic = rospy.get_param('~pointcloud')
    model = rospy.get_param('~model', 'cmu')

    resolution = rospy.get_param('~resolution', '432x368')
    resize_out_ratio = float(rospy.get_param('~resize_out_ratio', '4.0'))
    tf_lock = Lock()

    if not image_topic:
        rospy.logerr('Parameter \'camera\' is not provided.')
        sys.exit(-1)

    try:
        w, h = model_wh(resolution)
        graph_path = get_graph_path(model)

        rospack = rospkg.RosPack()
        graph_path = os.path.join(rospack.get_path('feature_recon'), graph_path)
    except Exception as e:
        rospy.logerr('invalid model: %s, e=%s' % (model, e))
        sys.exit(-1)

    pose_estimator = TfPoseEstimator(graph_path, target_size=(w, h))
    cv_bridge = CvBridge()

    image_sub = message_filters.Subscriber(image_topic, Image)
    point_sub = message_filters.Subscriber(cloud_topic, PointCloud2)
    ts = message_filters.ApproximateTimeSynchronizer([image_sub, point_sub], 10, 0.1)
    ts.registerCallback(callback_image)

 #   rospy.Subscriber(pointcloud, PointCloud2, callback_3d_mapping, queue_size=1, buff_size=2**24)

  #  rospy.Subscriber(image_topic, Image, callback_image, queue_size=1, buff_size=2**24)
    pub_pose = rospy.Publisher('~poses', Persons, queue_size=1)
    pub_img = rospy.Publisher('~image', Image, queue_size=1)

    rospy.loginfo('start+')
    rospy.spin()
    rospy.loginfo('finished')
