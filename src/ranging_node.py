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
from pozyx_ros.msg import DeviceRange as Range
from pozyx_ros.msg import DeviceRangeArray
import numpy as np

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
        self.pozyx.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_PRECISION, None)

        # functions have no return, change the device list in func
        # self.device_list = self.getDeviceList()
        self.device_list = None
        # self.getDeviceList()
        # print(self.device_list)

        # Publisher
        self.pubRange = rospy.Publisher('pozyx_range', DeviceRangeArray, queue_size=5)

        # thread lock
        # 0 => available, 1 => ranging, 2 => get device
        self.lock = False

        # to handle get device time
        self.last_search = rospy.Time.now()
        self.search_freq = 5

        # for moving average filter
        self.w_size = 10 # size is relative to speed of robot and accuracy
        self.device_ranges = {}
        self.device_lasttime = {}

    def getDeviceRange(self):

        if self.lock:
            return
        self.lock = True

        if self.device_list is None or len(self.device_list)==0:
            self.lock = False
            self.getDeviceList()
            return

        ranges = DeviceRangeArray()
        for device_id in self.device_list:
            # print('loop device list')
            device_range = DeviceRange()
            status = self.pozyx.doRanging(hex(device_id), device_range, None)
            if status == POZYX_SUCCESS:
                r = Range()
                r.tag_id = device_id
                r.header.stamp = rospy.Time.now()
                if device_id not in self.device_lasttime:
                    self.device_lasttime[device_id] = r.header.stamp
                if (r.header.stamp-self.device_lasttime[device_id]).to_sec() >= 0.5:
                    del self.device_ranges[device_id]
                self.device_lasttime[device_id] = r.header.stamp
                r.distance = self.moving_ave(device_range.distance, device_id)
                # r.RSS =
                ranges.rangeArray.append(r)
            else:
                rospy.logerr("pozyx ranging error")
                self.printErrorCode()

        if len(ranges.rangeArray) == 0:
            # Don't publish if there's no range found
            self.lock = False
            return

        # print("publish range")
        ranges.header.stamp = ranges.rangeArray[0].header.stamp
        self.pubRange.publish(ranges)
        self.lock = False
        if (rospy.Time.now()-self.last_search).to_sec() >= self.search_freq: # search every
            self.getDeviceList()

    def getDeviceList(self):

        print("search device")
        if self.lock:
            return
        self.lock = True
        print("search get lock")

        start_t = rospy.Time.now()
        self.pozyx.clearDevices()
        # self.pozyx.doDiscoveryAll(slots=3, slot_duration=0.1)
        status = self.pozyx.doDiscovery(discovery_type=PozyxConstants.DISCOVERY_ALL_DEVICES, slots=3, slot_duration=0.1)
        if status == POZYX_SUCCESS:
            device_list_size = SingleRegister()
            self.pozyx.getDeviceListSize(device_list_size)
            device_list = DeviceList(list_size=device_list_size[0])
            self.pozyx.getDeviceIds(device_list)
            self.device_list = device_list
            rospy.loginfo("found "+str(len(self.device_list))+" uwb devices")
            print(self.device_list)
        elif status == POZYX_TIMEOUT:
            self.device_list = None
            rospy.loginfo("found pozyx time out")
        else: # POZYX_FAILURE
            self.device_list = None
            rospy.loginfo("find pozyx device fail")
            self.printErrorCode()
            pass

        end_t = rospy.Time.now()
        print("Get Device list time: ", (end_t-start_t).to_sec())

        self.lock = False
        self.last_search = rospy.Time.now()

    def moving_ave(self, new_range, id):

        if id not in self.device_ranges:
            self.device_ranges[id] = np.full((self.w_size,), new_range)
            return np.mean(self.device_ranges[id])

        for i in range(1, self.w_size):
            self.device_ranges[id][i] = self.device_ranges[id][i-1]
        self.device_ranges[id][0] = new_range
        return np.mean(self.device_ranges[id])

    def printErrorCode(self):
        error_code = SingleRegister()
        status = self.pozyx.getErrorCode(error_code)
        if status == POZYX_SUCCESS:
            rospy.logerr(self.pozyx.getErrorMessage(error_code))
        else:
            rospy.logerr("error getting error msg")

if __name__ == '__main__':
    rospy.init_node('ranging_node',anonymous=False)
    pozyx_node = pozyx_node()

    try:
        # rospy.Timer(rospy.Duration(3), pozyx_node.getDeviceList)
        # add timer in the arguments of function if you would like to use rospy timer
        while not rospy.is_shutdown():
            pozyx_node.getDeviceRange()
            # rospy.sleep(0.01)
            #pozyx_node.()
            pass
    except rospy.ROSInterruptException:
        pass
