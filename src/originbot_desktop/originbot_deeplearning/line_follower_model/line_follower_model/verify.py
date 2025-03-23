from secrets import token_urlsafe
import cv2 as cv
import uuid
import os
import numpy as np
import torchvision
import torch
import torchvision.transforms as transforms
import torch.nn.functional as F
import glob
from PIL import Image
import PIL.Image
import torch.nn as nn
import torch.nn.init as init


model = torchvision.models.resnet18(pretrained=False)
model.fc = torch.nn.Linear(512,2)
model.load_state_dict(torch.load('/home/hcx/dev_ws/src/originbot_desktop/originbot_deeplearning/line_follower_model/best_line_follower_model_xy.pth'))
# model.load_state_dict(torch.load('/home/kairuiwang/code/line_tracking/best_steering_model_xy.pth'))
device = torch.device('cpu')
model = model.to(device)
model.eval()
x = torch.randn(1, 3, 224, 224, requires_grad=True)
torch_out = model(x)
torch.onnx.export(model,
                  x,
                  "line_tracking.onnx",
                  export_params=True,
                  opset_version=11,
                  do_constant_folding=True,
                  input_names=['input'],
                  output_names=['output'])
                #   dynamic_axes={'input' : {0 : 'batch_size'},    # variable length axes
                #                 'output' : {0 : 'batch_size'}})
mean = torch.Tensor([0.485, 0.456, 0.406])
std = torch.Tensor([0.229, 0.224, 0.225])

dir_path = '/home/hcx/dev_ws/src/originbot_desktop/originbot_deeplearning/line_follower_model/image_dataset'
image_paths = glob.glob(os.path.join(dir_path, '*.jpg'))
for i in range(len(image_paths)):
    print(image_paths[i])
    image_raw = cv.imread(image_paths[i])
    # x = int(os.path.basename(image_paths[i]).split("_")[1])
    # y = int(os.path.basename(image_paths[i]).split("_")[2])
    image = cv.resize(image_raw, (224,224))
    # x = int(x * 224 / 960)
    # cv.circle(image, (x,y), 5, (0,0,255), -1)
    # cv.imshow("capture image", image)
    # keyValue = cv.waitKey(-1)

    image_temp = Image.fromarray(cv.cvtColor(image,cv.COLOR_BGR2RGB))
    # image_temp = cv.cvtColor(image, cv.COLOR_BGR2RGB)
    # image = PIL.Image.open(image_paths[i])
    # image_temp = image.copy()
    # image_temp = transforms.functional.resize(image_temp, (224, 224))
    image_temp = transforms.functional.to_tensor(image_temp)
    # image_temp = image_temp.numpy()[::-1].copy()
    image_temp = image_temp.numpy().copy()
    image_temp = torch.from_numpy(image_temp)
    image_temp = transforms.functional.normalize(image_temp, [0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
    # image_temp.sub_(mean[:, None, None]).div_(std[:, None, None])
    xy = model(image_temp[None, ...]).detach().float().numpy().flatten()
    x = xy[0]
    y = xy[1]
    print(x, y)
    x = int((x * 112 + 112) * 960 / 224)
    y = int(224- (y * 112 + 112) * 223 / 224)
    print(x, y)
    cv.circle(image_raw, (x,y), 5, (0,0,255), -1)
    cv.imshow("capture image", image_raw)
    keyValue = cv.waitKey(-1)

    # cv.imshow("capture image", image)
    # while keyValue != 13:
    #     cv.setMouseCallback("capture image", mouse_callback, image)
    #     keyValue = cv.waitKey(1000)
