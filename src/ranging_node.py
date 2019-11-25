#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from pypozyx import *
from pypozyx.tools.device_list import *
from pypozyx.tools.discovery import *
from pypozyx.tools.version_check import *
from tf.transformations import quaternion_from_euler
from math import radians
from visualization_msgs.msg import Marker

class pozyx_node(object):
    def __init__(self):
        super(pozyx_node, self).__init__()

        # TODO: get device list, continue ranging and publishing

        '''
        self.anchors = rospy.get_param("~anchors")
        print self.anchors
        self.pub_poses = rospy.Publisher('~local_tag_pose', PoseStamped, queue_size=1)
        '''
        self.pozyx = PozyxSerial(get_first_pozyx_serial_port())
        self.pozyx.printDeviceInfo()

        self.device_list = self.getDeviceList()
        print(self.device_list)

    def getDeviceList(self):
        device_list_size = SingleRegister()
        self.pozyx.clearDevices()
        self.pozyx.doDiscoveryAll(slots=3, slot_duration=0.1)
        self.pozyx.getDeviceListSize(device_list_size)
        device_list = DeviceList(list_size=device_list_size[0])
        self.pozyx.getDeviceIds(device_list)
        return device_list

    def printErrorCode(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        if status == POZYX_SUCCESS:
            rospy.logerr(self.pozyx.getErrorMessage(error_code))
        else:
            rospy.logerr("error getting error msg")

if __name__ == '__main__':
    rospy.init_node('pozyx_node',anonymous=False)
    pozyx_node = pozyx_node()

    try:
        while not rospy.is_shutdown():
            pass
            #pozyx_node.()
    except rospy.ROSInterruptException:
        pass
