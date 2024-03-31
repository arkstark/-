import os

traintxt = open('train.txt','a')
valtxt = open('val.txt','a')
for img in os.listdir('train'):
    if "jpg" in img:
        traintxt.write(os.path.join('./Yolo-FastestV2-main/train/',img)+'\n')
        print(img)

    else:
        print(img,'error')
for img in os.listdir('val'):
    if "jpg" in img:
        valtxt.write(os.path.join('./Yolo-FastestV2-main/val/',img)+'\n')
        print(img)
    else:
        print(img,'error')
valtxt.close()
traintxt.close()
