# Laser_Obstacle_Avoidance

## 1. 准备工作
### 1.1. 安装Oradar驱动
下载源码，打开`oradar_ros`文件夹。 进入该文件夹，打开终端，输入：
```
mkdir bulid
cd build
cmake ..
make -j4
sudo make install
```

### 1.2.设置雷达的串口名
由于本项目用到了多个雷达，因此需要指定每一个雷达的串口映射。首先连接设备，打开终端输入
```
ll /dev/ttyACM*
```
这一代码将会返回所有ACM类的串口设备，如下：
```
crwxrwxrwx 1 root dialout 166, 0 Jan  3 16:47 /dev/ttyACM0
crwxrwxrwx 1 root dialout 166, 1 Jan  3 16:47 /dev/ttyACM1
crw-rw---- 1 root dialout 166, 2 Jan  3 17:24 /dev/ttyACM2
```
随后，查看串口设备的端口编号，输入：
```
udevadm info -a --name=/dev/ttyACM0 | grep devpath
```
返回：
```
    ATTRS{devpath}=="3"
    ATTRS{devpath}=="0"
```
即设备“/tty/ACM0"所在的端口编号为3， 依次查看其余两个设备的编号，随后进入`orada_ros`文件夹中，打开`oradar.rules`映射文件，修改内容为：
```
KERNEL=="ttyACM*",ATTRS{devpath}=="2", ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="55d4", MODE:="0777", SYMLINK+="oradar1"
KERNEL=="ttyACM*",ATTRS{devpath}=="3", ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="55d4", MODE:="0777", SYMLINK+="oradar2"
KERNEL=="ttyACM*",ATTRS{devpath}=="1", ATTRS{idVendor}=="1a86",ATTRS{idProduct}=="55d4", MODE:="0777", SYMLINK+="oradar3"
```
即端口类型为ttyACM, 指定ATTRS{devpath}为对应端口，以及设备对应的映射编号SYMLINK+。

### 1.3. 安装映射文件
打开终端，进入`oradar.rules`文件所在文件位置，输入：
```
sudo cp oradar.rules /etc/udev/rules.d/
```
随后，在终端输入
```
ll /dev/oradar1
```
返回
```
lrwxrwxrwx 1 root root 7 Jan  3 16:47 /dev/oradar1 -> ttyACM0
```
输入其他设备的编号（oradar2和oradar3）可查看其余两个设备的端口编号。
