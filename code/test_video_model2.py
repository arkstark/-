import os
import cv2
import time
import argparse
import numpy as np

import torch
import model.detector
import utils

if __name__ == '__main__':
    # 指定训练配置文件
    video_path = 0
    # 打开视频文件
    cap = cv2.VideoCapture(video_path)

    # 检查视频是否成功打开
    if not cap.isOpened():
        print('Error opening video file')

    parser = argparse.ArgumentParser()
    parser.add_argument('--data', type=str, default='data/coco.data',
                        help='Specify training profile *.data')
    parser.add_argument('--weights', type=str, default='weights/coco-290-epoch-1.000000ap-model.pth',
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
    fps = 10
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter(opt.output, fourcc, fps, (w, h))

    #储存右侧中心点坐标
    point_array = []

    while True:
        # 读取一帧
        ret, frame = cap.read()
        if not ret:
            break

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
            xright = x2
            yright = (y1+y2)/2
            point = np.array([xright,yright])
            point_array.append(point)



        out.write(frame)
        cv2.imshow('Object Detection', frame)
        if cv2.waitKey(1) == ord('q'):
            break
    print(point_array)
    # 释放资源
    cap.release()
    out.release()
    cv2.destroyAllWindows()