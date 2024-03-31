import argparse
import datetime
import os
import time

import pandas as pd
import cv2
import matplotlib.pyplot as plt
import torch

import model.detector
import utils
import serial
import openpyxl
import atexit


if __name__ == '__main__':
    # 指定训练配置文件
    video_path = 'D:\\NEXUS(History)\\OneDrive - arkstark\\桌面\\Yolo-FastestV2-main\\output.mp4'   # 指定相机地址
    #video_path = 0  # 指定本地文件

    # 打开视频文件
    cap = cv2.VideoCapture(video_path)
    fps = 0.0  # 初始化帧率为0
    frame_count = 0  # 初始化帧数为0
    start_time = time.time()  # 记录开始时间

    # 检查视频是否成功打开
    if not cap.isOpened():
        print('Error opening video file')

    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/coco.data',
                        help='Specify training profile *.data')
    parser.add_argument('--weights', type=str, default='weights/coco-390-epoch-1.000000ap-model.pth',
                        help='The path of the .pth model to be transformed')
    parser.add_argument('--output', type=str, default='out.wmv',
                        help='The path of output video')

    opt = parser.parse_args()

    cfg = utils.load_datafile(opt.data)
    assert os.path.exists(opt.weights), "请指定正确的模型路径"

    # 加载label names
    LABEL_NAMES = []
    with open(cfg["names"], 'r') as f:
        for line in f.readlines():
            LABEL_NAMES.append(line.strip())

    # 模型加载
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = model.detector.Detector(cfg["classes"], cfg["anchor_num"], True).to(device)
    model.load_state_dict(torch.load(opt.weights, map_location=device))

    # sets the module in eval node
    model.eval()

    # 视频输出配置
    _, frame = cap.read()
    h, w, _ = frame.shape
    fps = 30
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(opt.output, fourcc, fps, (w, h))

    #储存右侧中心点坐标
    xpoint_array = []
    ypoint_array = []
    xzero = 0
    xzero_array = []
    real_time = []
    # 初始化x和y坐标列表
    x = []
    y = []

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            break
        # 获取当前帧的时间戳（单位：秒）
        timestamp = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0

        frame_count += 1  # 增加帧数
        current_time = time.time()  # 记录当前时间
        if current_time - start_time >= 1.0:  # 每秒计算一次帧率
            fps = frame_count / (current_time - start_time)
            frame_count = 0  # 重置帧数
            start_time = current_time  # 更新开始时间

        cv2.putText(frame, "FPS: {:.2f}".format(fps), (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        cv2.imshow("Frame", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break  # 按下q键退出循环

        # 数据预处理
        res_img = cv2.resize(frame, (cfg["width"], cfg["height"]), interpolation=cv2.INTER_LINEAR)
        img = res_img.reshape(1, cfg["height"], cfg["width"], 3)
        img = torch.from_numpy(img.transpose(0, 3, 1, 2))
        img = img.to(device).float() / 255.0

        # 模型推理
        start = time.perf_counter()
        preds = model(img)
        end = time.perf_counter()
        forward_time = (end - start) * 1000.
        print("forward time:%fms" % forward_time)

        # 特征图后处理
        output = utils.handel_preds(preds, cfg, device)
        output_boxes = utils.non_max_suppression(output, conf_thres=0.3, iou_thres=0.4)

        scale_h, scale_w = h / cfg["height"], w / cfg["width"]

        # 绘制预测框并保存
        for box in output_boxes[0]:
            box = box.tolist()

            obj_score = box[4]
            category = LABEL_NAMES[int(box[5])]

            x1, y1 = int(box[0] * scale_w), int(box[1] * scale_h)
            x2, y2 = int(box[2] * scale_w), int(box[3] * scale_h)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 255, 0), 2)
            cv2.putText(frame, '%.2f' % obj_score, (x1, y1 - 5), 0, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, category, (x1, y1 - 25), 0, 0.7, (0, 255, 0), 2)

            #print(x1, y1, x2, y2)计算右侧中心点坐标
            xright = (x1+x2)/2
            # yright = (y1+y2)/2
            #point = np.array([xright,yright])
            # ypoint_array.append(yright)
            xpoint_array.append(xright)
            xzero += 1
            xzero_array.append(xzero)
            now = datetime.datetime.now()

        # 将时间戳和y坐标添加到列表中
        x.append(now)
        y.append(xright)



        # # 使用matplotlib绘制图形
        # plt.scatter(x, y)
        # plt.xlabel('Time (s)')
        # plt.ylabel('Y Coordinate')
        # plt.show()

        out.write(frame)
        cv2.imshow('Object Detection', frame)
        if cv2.waitKey(1) == ord('q'):
            break




# 串口通讯
#     ser = serial.Serial('COM3', 9600)  # replace 'COM3' with your COM port
#     data = []
#     def send_data(data):
#         for item in data:
#             ser.write(item)
#             data.append(item)
#     try:
#         while True:
#             new_data = input("Enter new data: ")
#             send_data(new_data)
#     except KeyboardInterrupt:
#         df = pd.DataFrame(data)
#         df.to_excel('data.xlsx', index=False)
#         ser.close()


    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()