# Keyence-SZ-16D-Driver (Python)

Python Driver for plot LiDAR/LaserScan ranges of Keyence SZ-16D with RS232/422 interface and 270 degrees of range.
Also provides the ROS Package to use this data on robots.

![AS_1917_L](https://user-images.githubusercontent.com/6139272/162611443-5d9e5e88-28f7-494e-b4b5-f2af5dca9033.png)

## Requirements

Tested used these libraries and ROS version:

* Python >= 3.8
* Numpy >= 1.22
* Matplotlib >= 3.5
* PySerial >= 3.5
* ROS 1 Noetic (if you want to use on ROS)

## Files

The main files of this package are:

* `src/laser.py`: python script to publish the LiDAR data on `/keyence_laser` topic.
* `src/plot_test.py`: python script to user compare the LiDAR data acquired from the SZ-16D Hardware directly using NumPy and Matplotlib.
* `src/sz16d.py`: python script to communicate with LiDAR. Returns a list of 751 ranges in Int or Float. This is the Driver!
* `config/laser_config.rviz`: Rviz config file to show the LiDAR data.
* `launch/lidar.launch`: Launch file that launch Rviz using `laser_config.rviz` configurations and `laser.py` script.

## Usage

Place the `keyence_laser` package on your catkin workspace and use:
`$ catkin_make`

After that you can use `roslaunch` or you can directly point to `laser.py` using:
`$ rosrun keyence_laser laser.py`
`$ rosrun rviz rviz`

If the Rviz config doesn't load, make sure to add the LaserScan on Display Types and to point the `/keyence_laser` topic to feed data. On Global Options, change the Fixed Frame `map` to `laser_frame`. You can see on images bellow the configuration.

## Images

Terminal (executing `roscore` and the scripts):

![Screenshot at Apr 10 02-00-29](https://user-images.githubusercontent.com/6139272/162610963-86ab6bc1-0f6f-4c17-a310-fb29c4066d3c.png)

Rviz showing the data capture from LiDAR:

![Screenshot at Apr 10 02-01-11](https://user-images.githubusercontent.com/6139272/162610973-c9163c72-9945-4692-8f88-480c06458cb8.png)


Rviz showing LiDAR data when I tried to block the Laser:

https://user-images.githubusercontent.com/6139272/162611125-5a763e35-e87c-4ec9-931c-903c342f0cba.mp4


Matplotlib using polar plot to show the LiDAR data (updating in real-time):

![Screenshot at Apr 10 04-48-44](https://user-images.githubusercontent.com/6139272/162610985-30755284-a3b4-4cb8-b22f-98e751c26183.png)


## Additional Hardware Used

In addition to the SZ-16D and its cable, other hardware components are required to use the laser in the computer, which are:

* RS422/485 converter to RS232 with Power Supply (used the Comm5 EF-232-485) [site](https://comm5.com.br/produtos/conversores/EF-232-485/);
* RS232 to USB converter (used the Comm5 1S-USB) [site](https://comm5.com.br/produtos/conversor-usb/1S-USB/);
* Power Supply 24Vdc (used the MeanWell LRS-35-24) [site](https://br.mouser.com/ProductDetail/MEAN-WELL/LRS-35-24?qs=vDxCgdWo2h9eBHrGk1xDdw%3D%3D).

The RS422/485 is used to connect the LiDAR RS422 interface to RS232 interface. The RS232 to USB is used to plug to your PC. And the Power Supply is used to power/feed the LiDAR.

## Issues Reported

Some problems detected on using the package:

 1. **[Errno5] Input/Output Error:** this problem appears when using the LiDAR on Virtual Machines. Try to use directly on PC. I think this problem is caused by the asynchronous communication between host and virtual machine USB;
 2. **Matplotlib doesn't show the plot:** make sure to install `pycairo` and `pygobject` package via: `pip install pycairo` and `pip install pygobject`. If appears a wheel problem due the toml package, make sure to install first both Cairo and GObject via brew or Apt-Get before install the Python dependencies.
