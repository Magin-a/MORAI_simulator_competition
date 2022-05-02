#!/usr/bin/env python
 # -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np


from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Float64MultiArray
from sklearn.cluster import DBSCAN
from morai_msgs.msg import CtrlCmd


class SCANCluster :

    def __init__(self) :

        self.scan_sub = rospy.Subscriber("/velodyne_points", PointCloud2, self.callback)
        self.cmd_pub = rospy.Publisher('/ctrl_cmd', CtrlCmd, queue_size=1)
        self.cluster_pub = rospy.Publisher("/clusters", Float64MultiArray, queue_size = 10)

        self.pc_np = None

        self.cluster_msg = Float64MultiArray()

        self.dbscan = DBSCAN(eps = 0.5, min_samples = 5)

    def pointcloud2_to_xyz(self,cloud_msg):

        point_list = []
        #cloud_msg안에 data를 x,y,z intensity, distance로 변환가능
        for point in pc2.read_points(cloud_msg,skip_nans=True):

            dist = np.sqrt(point[0]**2+point[1]**2+point[2]**2)
            # print('dist: ',dist)

            angle = np.arctan2(point[1],point[0])
            # print('angle: ',angle)

            if point[0]> 0 and point[2]> -1.3 and dist <30:
                point_list.append((point[0],point[1],point[2],point[3],dist,angle))


        point_np = np.array(point_list, np.float32)

        return point_np


    def callback(self, msg) :

        self.pc_np = self.pointcloud2_to_xyz(msg)

        pc_xy = self.pc_np[:, :2]
        # print(pc_xy)

        db = self.dbscan.fit_predict(pc_xy)

        n_cluster = np.max(db) + 1

        cluster_list = []
        
        for c in range(n_cluster) :
            c_tmp = np.mean(pc_xy[db==c, :], axis = 0)

            cluster_list += c_tmp.tolist()

        self.cluster_msg.data = cluster_list
        print(cluster_list)
        #수정코드 부분--------------------------------
        if 0 < min(cluster_list[0], cluster_list[2]) <10:
            rate = rospy.Rate(30)
            cmd = CtrlCmd()
            cmd.longlCmdType = 2
            cmd.velocity = 10
            steering_cmd = [ -0.4 , 0.4]
            cmd_cnts = 50

            while not rospy.is_shutdown():
                for i in range(2):
                    cmd.steering = steering_cmd[i]
                    rospy.loginfo(cmd)
                    for _ in range(cmd_cnts):
                        self.cmd_pub.publish(cmd)
                        rate.sleep()
        #----------------------------------------------
        self.cluster_pub.publish(self.cluster_msg)

if __name__ == '__main__' :
    rospy.init_node('cluster', anonymous = True)
    scancluster = SCANCluster()

    rospy.spin()
