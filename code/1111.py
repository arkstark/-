from torchsummary import summary
from torchvision.models import vgg16  # 以 vgg16 为例

myNet = vgg16()  # 实例化网络，可以换成自己的网络
summary(myNet, (3, 64, 64))  # 输出网络结构
