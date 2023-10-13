# coding:utf-8
#!/usr/bin/python
    
# Extract images from a bag file.
    
#PKG = 'beginner_tutorials'
import roslib;   #roslib.load_manifest(PKG)
import rosbag
import rospy
import cv2
import argparse
import os

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError
    
class ImageCreator():
    def __init__(self, bagfile, rgbpath, depthpath, rgbstamp, depthstamp):
        self.bridge = CvBridge()
        with rosbag.Bag(bagfile, 'r') as bag:
            for topic,msg,t in bag.read_messages():
                if topic == "/rgb/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"bgr8")
                        except CvBridgeError as e:
                            print(e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imshow("color", cv_image)
                        cv2.waitKey(1)
                        cv2.imwrite(rgbpath + image_name, cv_image)  #保存；

                        #写入时间戳
                        with open(rgbstamp, 'a') as rgb_time_file:
                            rgb_time_file.write(timestr+" rgb/"+image_name+"\n")


                elif topic == "/depth_to_rgb/image_raw": #图像的topic；
                        try:
                            cv_image = self.bridge.imgmsg_to_cv2(msg,"16UC1")
                        except CvBridgeError as e:
                            print(e)
                        timestr = "%.6f" %  msg.header.stamp.to_sec()
                        #%.6f表示小数点后带有6位，可根据精确度需要修改；
                        image_name = timestr+ ".png" #图像命名：时间戳.png
                        cv2.imwrite(depthpath + image_name, cv_image)  #保存；

                        #写入时间戳
                        with open(depthstamp, 'a') as depth_time_file:
                            depth_time_file.write(timestr+" depth/"+image_name+"\n")

    
if __name__ == '__main__':
    #rospy.init_node(PKG)
    parser = argparse.ArgumentParser(description="Grab the rgb and depth images from a ros bag")
    # parser.add_argument("--verbose", "-v", action='store_true', help='verbose_mode')
    # help = "The bag file"
    # parser.add_argument('bag', help=help, default='~/2023-10-09-13-28-22.bag')
    # help="The output folder"
    # parser.add_argument('output_folder', help=help, default='~/out_bag')
    # args = parser.parse_args()

    # bagfile = args.bag
    bagfile = '/home/lin/clothes/jeans_5_video_02.bag'
    output_folder = '/home/lin/out_bag/888'
    # rgb_path = args.output_folder + '/rgb/'
    # depth_path = args.output_folder + '/depth/'

    # rgb_timestamp_txt =  args.output_folder + "/rgb.txt"
    # depth_timestamp_txt = args.output_folder + "/depth.txt"

    rgb_path = output_folder + '/rgb/'
    depth_path = output_folder + '/depth/'

    rgb_timestamp_txt =  output_folder + "/rgb.txt"
    depth_timestamp_txt = output_folder + "/depth.txt"


    if not os.path.exists(rgb_path):
        os.makedirs(rgb_path)
    if not os.path.exists(depth_path):
        os.makedirs(depth_path)
    
    try:
        image_creator = ImageCreator(bagfile, rgb_path, depth_path, rgb_timestamp_txt, depth_timestamp_txt)
    except rospy.ROSInterruptException:
        pass

