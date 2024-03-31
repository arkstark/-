import os
import random
import shutil

image_Path = r'D:\POSTG\Yolo-FastestV2-main\train'  # 数据集路径
folder_txt_dir = ["train", "val"]


def Split_datasets(paths):
    '索引所有图片'
    images = []
    train_num_list = []
    val_num_list = []
    for path, folder, imgs in os.walk(paths):
        for img in imgs:
            if img.endswith(".jpg"):
                images.append(os.path.join(path, img))

def make_txt(folder_txt_dir):
    '创建train.txt和val.txt'
    for txt in folder_txt_dir:
        data_path = os.path.join(str(os.getcwd()), txt)
        image_list = os.listdir(txt)
        with open(file=txt + '.txt', mode='a+') as f:
            for name in image_list:
                if name.endswith(".jpg"):
                    item = os.path.join(data_path, name)
                    f.write(item)
                    f.write("\n")

    print('创建train.txt和val.txt成功')


if __name__ == '__main__':
    Split_datasets(image_Path)
    make_txt(folder_txt_dir)