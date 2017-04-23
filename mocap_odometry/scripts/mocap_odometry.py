#!/usr/bin/env python
# license reved for brevity
import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.broadcaster import TransformBroadcaster
import numpy as np
import socket

UDP_PORT = 50000

class UDPSocket:

    def __init__(self,PORT):
        self.port = PORT
        self.sock = None

    def accept(self, ip_address=''):
        self.sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
        self.sock.bind((ip_address, self.port))

    def receive(self,buffer_size = 1024):
        data, addr = self.sock.recvfrom(buffer_size) 

        return (data,addr)

class MocapOdometry:

    def __init__(self, PORT):
        """ Start up connection to the Neato Robot. """
        rospy.init_node('MocapOdometry')

        self.odomPub = rospy.Publisher('odom',Odometry, queue_size=10)
        self.pub = rospy.Publisher('hello',String, queue_size=10)
        self.odomBroadcaster = TransformBroadcaster()

        self.udp_socket = UDPSocket(PORT)
        self.udp_socket.accept()

    def quaternionFromEuler(self, ex, ey, ez):
        q = tf.transformations.quaternion_from_euler(ex, ey, ez)
        quat = Quaternion(*q)
        return quat

    def quaternionFromRotation(self, rotation_as_array):

        r = rotation_as_array
        rotation = np.array([[r[0],   r[1],   r[2], 0],
                             [r[3],   r[4],   r[5], 0],
                             [r[6],   r[7],   r[8], 0],
                             [0,      0,      0,    1]])

        q = tf.transformations.quaternion_from_matrix(rotation)
        qmatrix = tf.transformations.quaternion_matrix(q)

        quat = Quaternion(*q)
        return quat

    def parseDataToSE3(self, data):

        # print "data: ", data

        tokens = data.split(' ')

        # print "tokens: ", tokens

        array = map(lambda x: float(x), tokens)

        # print "array: ", array

        p = array[:3]
        q = self.quaternionFromRotation(array[3:]) #self.quaternionFromEuler(*array[3:])

        # print "pose: ", p, q

        return (p,q)

    def spin(self):

        self.x = 0 
        self.y = 0
        self.z = 0
        self.qx = 0
        self.qy = 0
        self.qz = 0
        self.qw = 1
        then = rospy.Time.now()

        qt = self.quaternionFromEuler(0,0,0)

        # things that don't ever change
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')

        # odom.twist.twist.linear.x = dx/dt
        # odom.twist.twist.angular.z = dth/dt


        rate = rospy.Rate(500) # 10hz
        while not rospy.is_shutdown():


            data, _ = self.udp_socket.receive()

            # prepare tf from base_link to odom
            p, quaternion = self.parseDataToSE3(data)
            self.x = p[0]
            self.y = p[1]
            self.z = p[2]
            self.qx = quaternion.x
            self.qy = quaternion.y
            self.qz = quaternion.z
            self.qw = quaternion.w
            now = rospy.Time.now()
            # prepare odometry
            odom.header.stamp = rospy.Time.now()
            odom.pose.pose.position.x = self.x
            odom.pose.pose.position.y = self.y
            odom.pose.pose.position.z = self.z
            odom.pose.pose.orientation = quaternion

            # publish everything
            child_frame_id = "base_link"
            parent_frame_id = "odom"
            self.odomBroadcaster.sendTransform((self.x, self.y, self.z), (-quaternion.x, -quaternion.y, -quaternion.z, quaternion.w),
                    then, child_frame_id, parent_frame_id ) 
            print '\n\npos:',(self.x, self.y, self.z),'\n','quat:', (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
            self.odomPub.publish(odom)
            rate.sleep()

if __name__ == '__main__':

    mocap_odometry = MocapOdometry(50000)

    try:
        mocap_odometry.spin()
    except rospy.ROSInterruptException:
        pass
