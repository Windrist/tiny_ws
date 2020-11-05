## Movement
Type of motion: Two wheeled differential drive <br>
Motor type: Stepper motor <br>
Stepper motor name: 28BYJ-48 <br>
Driver: ULN2003 <br>
Power input: 5V <br>
Max speed of stepper: 19rpm (check again with your input) <br>

## Step to step
#### Control Stepper Motor with Arduino <br> 
* [28BYJ-48 Stepper Motor with ULN2003 Driver and Arduino Tutorial](https://www.makerguides.com/28byj-48-stepper-motor-arduino-tutorial/)
#### ROS Arduino <br>
* [Installation](http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup) <br>
* [Tutorials](http://wiki.ros.org/rosserial_arduino/Tutorials) <br>
#### Control Stepper Motor with ROS Arduino <br>
1. Download and install in your workspace<br>
```
$ cd [your_ws]/src
$ git clone https://github.com/DuyNamUET/movement
$ cd ..
$ catkin_make
```
2. Run <br>
* Firstly, check your port Arduino and change parameter on launch file. Here, it is ```/dev/ttyACM0``` <br>
```
<launch>
    <node pkg="rosserial_python" type="serial_node.py" name="serial_node"
            args="/dev/ttyACM0" respawn="true" />
    <node pkg="movement" type="control_directly" name="control_directly" args=""
            output="screen" />
</launch>
```
* Then:
```
$ roslaunch movement run_motor.launch
```