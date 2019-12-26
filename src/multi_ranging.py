#!/usr/bin/env python

import rospy
from pypozyx import *
from pypozyx.tools.device_list import *
from pypozyx.tools.discovery import *
from pypozyx.tools.version_check import *
from pozyx_ros.msg import DeviceRange as Range
from pozyx_ros.msg import DeviceRangeArray
from std_msgs.msg import Bool
import numpy as np

class pozyx_node(object):
    def __init__(self):
        super(pozyx_node, self).__init__()

        self.my_id = rospy.get_param(rospy.get_param("~id_param_name"))
        rospy.loginfo("my id: 0x%0.4x" % self.my_id)

        self.port_list = get_pozyx_ports()
        self.pozyx = None
        if len(self.port_list) == 0:
            rospy.logfatal("no device attached")
            raise RuntimeError("No pozyx device attached")

        for port in self.port_list:
            try:
                rospy.loginfo("trying " + str(port))
                self.pozyx = PozyxSerial(port)
                network_id = NetworkID()
                self.pozyx.getNetworkId(network_id)
                if network_id == self.my_id:
                    rospy.loginfo("found my port: 0x%0.4x" % self.my_id)
                    break
                else:
                    rospy.logwarn("not my port")
                    self.pozyx.ser.close()
            except SerialException:
                rospy.logwarn("Got Serial Exception")
        if self.pozyx == None:
            rospy.logerr("No device found with certain ID")
            raise RuntimeError("No device found with certain ID")

        self.pozyx.printDeviceInfo()
        self.pozyx.setRangingProtocol(PozyxConstants.RANGE_PROTOCOL_PRECISION, None)

        self.device_list = None
        self.dest_device_list = {}
        self.id_param_list = rospy.get_param("~dest_ids_name")
        for param in self.id_param_list:
            self.dest_device_list[param] = rospy.get_param(param)

        # Publisher
        self.pubRange = rospy.Publisher('ranges', DeviceRangeArray, queue_size=5)
        self.pub_com = rospy.Publisher('/ranging', Bool, queue_size=1)
        self.sub_com = rospy.Subscriber('/ranging', Bool, self.com_cb, queue_size=1)
        self.ranging = False

        # thread lock
        # 0 => available, 1 => ranging, 2 => get device
        self.lock = 0


    def getDeviceRange(self):

        if self.lock:
            rospy.loginfo("locked return")
            return
        self.lock = True

        if self.ranging:
            rospy.sleep(0.1)

        data = Bool()
        data.data = True
        self.pub_com.publish(data)
        ranges = DeviceRangeArray()
        for device in self.dest_device_list:
            rospy.loginfo("ranging %s 0x%0.4x" % (device, self.dest_device_list[device]))
            device_range = DeviceRange()
            status = self.pozyx.doRanging(self.dest_device_list[device], device_range, None)
            if status == POZYX_SUCCESS:
                print(device_range)
                r = Range()
                r.tag_id = self.dest_device_list[device]
                r.header.stamp = rospy.Time.now()
                r.distance = device_range.distance
                rospy.loginfo("0x%0.4x : %d" %(self.dest_device_list[device], r.distance))
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
        data = Bool()
        data.data = False
        self.pub_com.publish(data)

    def getDeviceList(self):

        rospy.loginfo("search device")
        if self.lock:
            return
        self.lock = True


        start_t = rospy.Time.now()
        self.pozyx.clearDevices()
        
        #discover_all_devices(self.pozyx)        
        #end_t = rospy.Time.now()
        #print("Get Device list time: ", (end_t-start_t).to_sec())
        #self.lock = False
        #return

        # self.pozyx.doDiscoveryAll(slots=3, slot_duration=0.1)
        status = self.pozyx.doDiscoveryAll(slots=5, slot_duration=0.1)
        if status == POZYX_SUCCESS or status == POZYX_TIMEOUT:
            device_list_size = SingleRegister()
            self.pozyx.getDeviceListSize(device_list_size)
            device_list = DeviceList(list_size=device_list_size[0])
            self.pozyx.getDeviceIds(device_list)
            self.device_list = device_list
            rospy.loginfo("found "+str(len(self.device_list))+" uwb devices")
            #print(self.device_list)
        #elif status == POZYX_TIMEOUT:
        #    self.device_list = None
        #    rospy.logerr("found pozyx time out")
        else: # POZYX_FAILURE
            self.device_list = None
            rospy.logerr("find pozyx device fail")
            self.printErrorCode()
            #pass

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

    def com_cb(self, msg):
        self.ranging = msg.data

        

if __name__ == '__main__':
    rospy.init_node('multi_ranging',anonymous=False)
    freq = rospy.get_param("~ranging_freq")

    timer = rospy.timer.Rate(freq)
    try:
        pozyx_node = pozyx_node()
        while not rospy.is_shutdown():
            pozyx_node.getDeviceRange()
            timer.sleep()
    except RuntimeError:
        rospy.logfatal("get runtime error")
    except rospy.ROSInterruptException:
        pass
