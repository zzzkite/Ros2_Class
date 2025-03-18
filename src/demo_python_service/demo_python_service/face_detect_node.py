import os
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory #获得功能包share的绝对路径
import rclpy
from rclpy.node import Node
from service_interface.srv import FaceDetector
from cv_bridge import CvBridge
import time
from rcl_interfaces.msg import SetParametersResult #设置参数回调函数需要用到的库

class FaceDetectNode(Node):
    def __init__(self):
        super().__init__('face_detect_node')
        self.service_ = self.create_service(FaceDetector, 'face_detect', self.face_detect_callback)#实例化创建服务
        self.bridge = CvBridge() #实例化CVBRIDGE
        # self.number_of_times_to_upsample = 1
        # self.model = 'hog'
        self.declare_parameter('number_of_times_to_upsample',1)
        self.declare_parameter('model','hog')
        self.number_of_times_to_upsample = self.get_parameter('number_of_times_to_upsample').value
        self.model = self.get_parameter('model').value
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
        self.add_on_set_parameters_callback(self.parameters_callback)

    def parameters_callback(self, parameters):
        for parameter in parameters:
            self.get_logger().info(f"{parameter.name}->{parameter.value}")
            if parameter.name == 'number_of_times_to_upsample':
                self.number_of_times_to_upsample = parameter.value
            if parameter.name == 'model':
                self.model = parameter.value
        return SetParametersResult(successful=True) #参数设置回调函数要求有这个返回值

    def face_detect_callback(self, request, response):
        #检测是否传入正确
        if request.image.data:
            cv_image = self.bridge.imgmsg_to_cv2(request.image)
            self.get_logger().info(f'接收到传入图像，开始解析')
        else:
            cv_image = cv2.imread(self.default_image_path)
            self.get_logger().info(f'传入图像为空，使用默认图像')

        start_time = time.time() #获取当前时间
        self.get_logger().info(f'加载图像完成，开始识别')
        face_locations = face_recognition.face_locations(cv_image, number_of_times_to_upsample=self.number_of_times_to_upsample, model=self.model)
        response.use_time = time.time() - start_time
        self.get_logger().info(f'识别完成，耗时{response.use_time}')
        response.number = len(face_locations)
        for top,bottom,right,left in face_locations:
            response.top.append(top)
            response.bottom.append(bottom)
            response.right.append(right)
            response.left.append(left)
        return response #必须返回response


def main():
    rclpy.init()
    node = FaceDetectNode()
    rclpy.spin(node)
    rclpy.shutdown()
        