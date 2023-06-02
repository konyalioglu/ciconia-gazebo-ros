#CICONIA ROS PACKAGE

A ROS Noetic package for the hybrid VTOL aircraft called Ciconia. It was designed to be used in parcel delivery applications and more details can be found in the folloing link: https://web.itu.edu.tr/altuger/

## NVIDIA JETSON ROS CONTROL

Establish connection using ssh for Jetson 

```
ssh nvidia@ipv4-address
```

Use 'nvidia' as password.


Check the docker images by using command,
```
sudo docker images
```

You will see the list of images exist in the machine. Then, run the latest version of the docker image using command below,

```
sudo docker run -it --rm --runtime nvidia --device=/dev/video0 --device=/dev/i2c-1 --device=/dev/ttyACM0  --network host docker.io/ciconia:noetic-v3
```
You have to add serial, i2c and video devices to establish connection for ArduPilot, related sensor and camera module.

Change the directory to '/ciconia' and source the catkin package,
```
cd ciconia

source  devel/setup.bash
```

To launch the indoor altitude test use the command below,

```
roslaunch ciconia_indoor_test_launch test2.launch
```

That will launch '/alt_controller', '/alt_estimation', '/alt_logger' and '/mavros' nodes.

The related Python codes can be found in the package directory referencing to test2.launch file.

Also, if you'd like to see list of the topic published by the related nodes. You can open second docker terminal by using command below,

```
docker exec -it <container_id> bash
```

Retrieve the <container_id> using 

```
sudo docker ps
```
Then, you can see the topics to be publishin by the nodes by using following command,

```
rostopic list
```

You can also change the controller type and controller rate by editting alt_controller_config.yaml located in the directory of 'ciconia_indoor_test_launch/config'. There are two controller options which are MPC and PID. The parameter of them can be changed modifying the indoor_control2.py.


A simple simulation can be performed by using command below,

```
roslaunch ciconia_indoor_test_launch sitl.launch
```


IMPORTANT! In the controller node (indoor_control2.py) the altitude control set_points are set regarding the PWM signal obtained from the Channel 6. As it can be seen below,

```
def _rc_in_handler(self, msg):
    if len(msg.channels) > 4:
        self.set_point_settling = msg.channels[5]
        self.throttle_in = (msg.channels[2] - 1000) / 10
        if self.set_point_settling < 1300:
            self.ref_alt = 0.10
        elif self.set_point_settling > 1300 and self.set_point_settling < 1700:
            self.ref_alt = 0.20
        elif self.set_point_settling > 1700:
            self.ref_alt = 0.0

```

Therefore, if you'd like to use another RC controller, you must check RC switches correspond to RC channel inputs. 


Also, check the switch related with the mode switch. To control the aircraft it should be in guided mode.


If you would like to make changes in the packages, you have use 'catkin build' command and don't forget to commit docker image running in the main terminal.


## GAZEBO SIMULATIONS AND OTHERS

'ciconia_gazebo' package is created to perform aircraft simulations. It depends some other packages such as 'ciconia_control', 'ciconia_sensors'. It consists of some Gazebo plugins written in C++ which can be found in plugins directory. 'urdf' directory includes robot description files and 'scripts' file contains necessary Python file such as physcis.py (aerodynamic gyroscopic, gravitational, propulsion model), control.py (main controller node for the simulation), processImage.py (for Aruco application).


The simulation can be run by using following command,
```
roslaunch ciconia_gazebo vtol.launch
```

Implementation of state_estimator simulation is not ready yet.


In the ciconia_sensors package, sensor models like gyroscope, magnetometer, barometer and gps can be found to be used in Gazebo simulations. 


Libraries for MPC, LQR and PID written in Python can be found in ciconia_control package.

 
Some kalman filters and ahrs applications can be found in ciconia_navigation package.


At last, a serialization package is created to establish serial communication between a STM32 microcontroller and NVIDIA Jetson. The package reads SBUS data and sends PWM signals for BLDC motor to be driven. For modification, serialization.py file can be analyzed.


