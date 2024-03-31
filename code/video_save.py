
"""
    使用python实现：读取USB摄像头的画面
"""
# 导入CV2模块
import cv2

def read_usb_capture():
    # 选择摄像头的编号
    cap = cv2.VideoCapture(0)
    # 添加这句是可以用鼠标拖动弹出的窗体
    cv2.namedWindow('real_img', cv2.WINDOW_NORMAL)
    # # .flv 格式 , 25为 FPS 帧率， （640,480）为大小
    # fourcc = cv2.VideoWriter_fourcc(*'flv1')
    # out = cv2.VideoWriter('output.flv', fourcc, 25, (640, 480))

    # .mp4格式 , 25为 FPS 帧率， （640,480）为大小
    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    out = cv2.VideoWriter('output2.mp4', fourcc, 30, (640, 480))

    # # .avi格式 , 25为 FPS 帧率， （640,480）为大小
    # fourcc = cv2.VideoWriter_fourcc(*'XVID')
    # out = cv2.VideoWriter('output.avi', fourcc, 25, (640, 480))

    while(cap.isOpened()):
        # 读取摄像头的画面
        ret, frame = cap.read()

        # 进行写操作
        out.write(frame)
        # 真实图
        cv2.imshow('real_img', frame)

        # 按下'q'/esc就退出
        # if cv2.waitKey(1) & 0xFF == ord('q'):
        if cv2.waitKey(1) & 0xFF == 27:
            break
    # 释放画面
    cap.release()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    read_usb_capture()
