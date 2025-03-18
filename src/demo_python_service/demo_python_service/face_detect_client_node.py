import os
import cv2
from ament_index_python.packages import get_package_share_directory #获得功能包share的绝对路径
import rclpy
from rclpy.node import Node
from service_interface.srv import FaceDetector
from cv_bridge import CvBridge
from rcl_interfaces.msg import Parameter,ParameterValue,ParameterType
from rcl_interfaces.srv import SetParameters

class FaceDetectClientNode(Node):
    def __init__(self):
        super().__init__('face_detect_client_node')
        self.client_ = self.create_client(FaceDetector, 'face_detect')
        self.bridge = CvBridge() #实例化CVBRIDGE
        self.default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
        self.image = cv2.imread(self.default_image_path)
        
    def send_request(self):
        #1.判断服务端是否在线
        while self.client_.wait_for_service(timeout_sec=1.0) is False:
            self.get_logger().info(f'检测到服务端在线')
        #2.构造请求
        request = FaceDetector.Request()
        request.image = self.bridge.cv2_to_imgmsg(self.image)
        #3.发送请求并等待处理完成(方法一：定义回调函数)
        future = self.client_.call_async(request)
        future.add_done_callback(self.result_callback)

        #方法二：直接用rclpy的方法，但会阻塞spin，效果不如方法一
        # rclpy.spin_until_future_complete(self,future) #等待服务器返回响应
        # response = future.result() #获取响应结果
        # self.get_logger().info(f'接收到响应，共有{response.number}张人脸，耗时{response.use_time}s')
        # self.show_response(response)
        

    def show_response(self,response):
        for i in range(response.number):
            top = response.top[i]
            bottom = response.bottom[i]
            right = response.right[i]
            left = response.left[i]
            cv2.rectangle(self.image, (left,top), (right,bottom), (0,0,255), 4)
        cv2.imshow('Face Detect Result', self.image)
        cv2.waitKey(0) #也是阻塞态，会影响spin，但一次响应够用了

    def call_set_parameter(self, parameters):
        # 1. 创建客户端，等待服务上线
        update_param = self.create_client(SetParameters, '/face_detect_node/set_parameters')
        
        # 等待服务端在指定的时间内上线，超时后会返回警告信息
        while not update_param.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("等待服务端上线...")
        
        # 服务上线后，输出信息
        self.get_logger().info("服务已上线，准备调用")
        
        # 2. 创建请求对象
        request = SetParameters.Request()
        
        # 将传入的参数设置到请求对象中
        request.parameters = parameters
        
        # 3. 异步调用服务端更新参数
        future = update_param.call_async(request)
        
        # 等待请求完成，直到返回结果
        rclpy.spin_until_future_complete(self, future)
        
        # 返回服务端返回的结果
        return future.result()

    def update_detect_model(self, model='hog'):
        # 1. 构造参数对象，设置参数的名称为 'model'
        param = Parameter()
        param.name = 'model'
        
        # 2. 创建参数值对象，设置其值为传入的模型类型
        param_value = ParameterValue()
        param_value.string_value = model
        param_value.type = ParameterType.PARAMETER_STRING
        
        # 将参数值对象赋值给参数对象
        param.value = param_value  
        
        # 3. 调用函数更新参数，并等待服务返回结果
        response = self.call_set_parameter([param])
        
        # 遍历服务返回的结果，并记录日志
        for result in response.results:
            self.get_logger().info(f"设置参数结果{result.successful}{result.reason}")


    def result_callback(self,future):
            response = future.result() #获取响应结果
            self.get_logger().info(f'接收到响应，共有{response.number}张人脸，耗时{response.use_time}s')
            self.show_response(response)


def main():
    rclpy.init()
    node = FaceDetectClientNode()
    node.update_detect_model('hog')
    node.send_request()
    node.update_detect_model('cnn')
    node.send_request()
    rclpy.spin(node)
    rclpy.shutdown()
        