import rospy
import airsim
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
import threading
import time
from copy import copy
from cv_bridge import CvBridge





class AirSimMultiRotor:

    def __init__(self):
        # connect to the AirSim simulator
        self.client = airsim.MultirotorClient()
        self.client.confirmConnection()
        self.bridge = CvBridge()
        self.image_pub_left = rospy.Publisher("/stereo_robot/mobile_base/camera/left/image_raw", Image, queue_size=5)
        self.image_pub_right = rospy.Publisher("/stereo_robot/mobile_base/camera/right/image_raw", Image, queue_size=5)
        self.compressed_image_pub_left = rospy.Publisher("/stereo_robot/mobile_base/camera/left/image_raw/compressed", CompressedImage, queue_size=5)
        self.compressed_image_pub_right = rospy.Publisher("/stereo_robot/mobile_base/camera/right/image_raw/compressed", CompressedImage, queue_size=5)
        self.camera_info_pub_left = rospy.Publisher("/stereo_robot/mobile_base/camera/left/camera_info", CameraInfo, queue_size=5)
        self.camera_info_pub_right = rospy.Publisher("/stereo_robot/mobile_base/camera/right/camera_info", CameraInfo, queue_size=5)

        self.left_cnt_ = 0
        self.right_cnt_ = 0

        self.image_timer = threading.Timer(0.1, self.image_response_cb)
        self.image_timer.start()

        self.camera_info_msg = CameraInfo()
        self.camera_info_msg.height = 240
        self.camera_info_msg.width = 320
        self.camera_info_msg.distortion_model = "plumb_bob"
        self.camera_info_msg.D = []
        self.camera_info_msg.K = [160.0, 0.0, 160.0, 0.0, 160.0, 120.0, 0.0, 0.0, 1.0]
        self.camera_info_msg.R = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.camera_info_msg.P = [160.0, 0.0, 160.0, 0.0, 0.0, 160.0, 120.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.camera_info_msg.binning_x = 0
        self.camera_info_msg.binning_y = 0
        self.camera_info_msg.roi.x_offset = 0
        self.camera_info_msg.roi.y_offset = 0
        self.camera_info_msg.roi.height = 0
        self.camera_info_msg.roi.width = 0
        self.camera_info_msg.roi.do_rectify = False


    def get_img_msg_from_response(self, response,  frame_id):
        img1d = np.frombuffer(response.image_data_uint8, dtype=np.uint8)
        img_msg = Image()

        # img_msg.header.stamp = rospy.Time.now()
        img_msg.header.frame_id = frame_id
        img_msg.height = response.height
        img_msg.width = response.width
        img_msg.encoding = 'bgr8'
        img_msg.is_bigendian = 0
        img_msg.data = img1d.tobytes()
        img_msg.step = img_msg.width * 3  # assuming bgr8


        image_array = np.array(img1d).reshape((response.height, response.width, 3))
        compressed_img_msg = CompressedImage()
        compressed_img_msg.header.stamp = img_msg.header.stamp
        compressed_img_msg.header.frame_id = img_msg.header.frame_id
        compressed_img_msg.format = "rgb8; jpeg compressed bgr8"  # Set the compression format
        compressed_img_msg.data = self.bridge.cv2_to_compressed_imgmsg(image_array, dst_format='jpeg').data

        camera_info_msg = copy(self.camera_info_msg)
        camera_info_msg.header.stamp = img_msg.header.stamp
        camera_info_msg.header.frame_id = img_msg.header.frame_id
        # print(type(compressed_img_msg))

        return img_msg, compressed_img_msg, camera_info_msg

    def image_response_cb(self):
        self.client.simPause(True)
        requests = [airsim.ImageRequest("front_left", airsim.ImageType.Scene, False, False),  # left
                    airsim.ImageRequest("front_right", airsim.ImageType.Scene, False, False)]  # right

        responses = self.client.simGetImages(requests)
        self.client.simPause(False)

        left_img_msg, compressed_left_img_msg, left_camera_info_msg = self.get_img_msg_from_response(responses[0], "front_left")
        right_img_msg, compressed_right_img_msg, right_camera_info_msg = self.get_img_msg_from_response(responses[1], "front_right")

        self.image_pub_left.publish(left_img_msg)
        self.image_pub_right.publish(right_img_msg)
        self.compressed_image_pub_left.publish(compressed_left_img_msg)
        self.compressed_image_pub_right.publish(compressed_right_img_msg)
        self.camera_info_pub_left.publish(left_camera_info_msg)
        self.camera_info_pub_left.publish(right_camera_info_msg)

        self.image_timer = threading.Timer(0.1, self.image_response_cb)
        self.image_timer.start()


if __name__ == "__main__":
    rospy.init_node('airsim_node', anonymous=True)
    airsim_multirotor = AirSimMultiRotor()
    rospy.spin()