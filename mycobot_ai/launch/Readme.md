### Tips：

1. 请使用与机械臂相同型号的文件名文件。

2. 请先执行以下操作:
   + 1. 打开一个新的终端
   + 2. 输入命令
```bash
chmod +x /home/h/catkin_mycobot/src/mycobot_ros/mycobot_communication/scripts/xxx.py
							# 此处为各个新增文件的文件名
```
3. jetson nano的文件还没有使用机械臂进行过测试，可能存在问题。

4. 数莓派版本的使用:
   + 1. 打开VScode,新建一个文件，复制以下内容（请确保电脑与数莓派机械臂已经连接）并运行
	```bash
	from pymycobot import MyCobotSocket

	mc = MyCobotSocket("192.168.10.10","9000")
	mc.connect()
	```

   + 2. 打开“网络与internet”设置
	更改适配器选项
	右键打开数莓派的以太网属性
	打开 “internet协议版本4” 的属性
	选择 “使用下面的IP地址”
	IP地址为 ：  192.168.10.100 （最后一位非10都可）
	子网掩码为： 255.255.255.0
	确认
	
	
###ENGLISH

###Tips:

1. Please use the file name file of the same model as the robot arm.

2. Please do the following first:
    + 1. Open a new terminal
    + 2. Enter the following command
```bash
chmod +x /home/h/catkin_mycobot/src/mycobot_ros/mycobot_communication/scripts/xxx.py
										# Here is the file name of each new file
````
3. The files of jetson nano have not been tested with the robotic arm, there may be problems.

4. Use of Raspberry Pi version:
    + 1. Open VScode, create a new file, copy the following content (please make sure the computer and the Raspberry Pi robotic arm have been connected) and run
```bash
from pymycobot import MyCobotSocket

mc = MyCobotSocket("192.168.10.10","9000")
mc.connect()
````

    + 2. Open "Network and internet" settings
Change adapter options
Right-click to open the Ethernet properties of the Raspberry Pi
Open the properties of "internet protocol version 4"
Select "Use the following IP address"
The IP address is: 192.168.10.100 (the last digit is not 10)
The subnet mask is: 255.255.255.0
	confirm






