#!/usr/bin/env python
import os
import rospy
from std_msgs.msg import String, ByteMultiArray
from darknet_ros_msgs.msg import BoundingBox
from darknet_ros_msgs.msg import BoundingBoxes
import struct
import numpy as np

class ZCU_handler:
    def __init__(self):
        self.pub_bbx = rospy.Publisher('/darknet_ros/bounding_boxes1', BoundingBoxes, queue_size=1)
        rospy.Subscriber("bbox_publish", ByteMultiArray, self.callback)
        self.class_name = {
                    0:"person",
                    1:"wheelchair",
                    2:"shoppingcart",
                    3:"babybuggy",
                    4:"personR",
                    5:"personL",
                    6:"personF",
                    7:"personB",
                }

    def callback(self, data):
        receive_dic = {}
        bbox_list = []
        id_list = []
        #rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)

        result = np.array(data.data, dtype = np.uint8)

        receive_dic["time_stamp"] = struct.unpack("q", bytes(result[:8]))[0]
        receive_dic["is_iframe"] = struct.unpack("?", bytes(result[8:9]))[0]
        receive_dic["bbox_size"] = struct.unpack("q", bytes(result[9:17]))[0]

        BBXes = BoundingBoxes()
        for i in range(struct.unpack("q", bytes(result[9:17]))[0]):
            base = 17 + 24 * i
            # label||bbox.x||bbox.y||bbox.weight||bbox.height||bbox.box_id
            _tmp_list = struct.unpack("iffffi", bytes(result[base:base+24]))
            #bbox_list.append(_tmp_list)
            
            #-------------------
            BBX = BoundingBox()
            BBX.Class = self.class_name[_tmp_list[0]]
            BBX.xmin = max(0, int(_tmp_list[1]-(_tmp_list[3]//2)))
            BBX.ymin = max(0, int(_tmp_list[2]-(_tmp_list[4]//2)))
            BBX.xmax = max(0, int(_tmp_list[1]+(_tmp_list[3]//2)))
            BBX.ymax = max(0, int(_tmp_list[2]+(_tmp_list[4]//2)))
            id_list.append(_tmp_list[5])
            BBXes.bounding_boxes.append(BBX)
            #-------------------
        os.system("clear")
        print("="*30)
        for i, bbx in enumerate(BBXes.bounding_boxes):
            print(bbx)
            if id_list[i]!=-1:
                print("id: {}".format(id_list[i]))
            print("-"*15)
        #print("-----")
        #print(receive_dic)
        print("="*30)


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('apple', anonymous=True)

    # rospy.Subscriber("bbox_publish", ByteMultiArray, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('apple', anonymous=True)
    Z = ZCU_handler()
    print("start")
    rospy.spin()


    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        r.sleep()
