import os
import face_recognition
import cv2
from ament_index_python.packages import get_package_share_directory #获得功能包share的绝对路径

def main():
    # 获得图片的真实路径，可以利用os库来帮助补全路径，会自动补全/等
    default_image_path = os.path.join(get_package_share_directory('demo_python_service'), 'resource/default.jpg')
    # 使用cv2来加载图片
    image = cv2.imread(default_image_path)
    # 检测人脸
    face_locations = face_recognition.face_locations(image, number_of_times_to_upsample=1, model='hog')
    # 绘制人脸框
    for top,bottom,right,left in face_locations:
        cv2.rectangle(image, (left,top), (right,bottom), (0,0,255), 4)
    
    # 结果显示
    cv2.imshow('Face detect result', image)
    cv2.waitKey(0)
