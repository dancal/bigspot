## gazebo
### roslaunch bigspot run_gazebo_only.launch
### roslaunch bigspot_joystick ramped_keyboard.launch
### ~~roslaunch bigspot_description controller_gazebo.launch~~


## real
# roslaunch bigspot run_robot_real.launch
# roslaunch bigspot_description controller.launch
# roslaunch bigspot_joystick ramped_joystick.launch

## etc
# roslaunch bigspot_mpu6050 mpu.launch
# roslaunch bigspot_mpu9250 mpu.launch
# roslaunch bigspot_lcd lcd.launch
# roslaunch bigspot_rgb rgb.launch
# roslaunch bigspot_ultrasonic ultrasonic.launch


## packages
### sudo apt install ros-noetic-joy
### sudo apt install ros-noetic-imu-tools
### sudo apt install libgpiod2
### sudo apt install ros-noetic-ros-controllers

### pip3 install pygame
### pip3 install adafruit-circuitpython-rgbled
### pip3 install adafruit-circuitpython-motor
### pip3 install adafruit-circuitpython-servokit
### pip3 install adafruit-circuitpython-rgbled / https://github.com/adafruit/Adafruit_CircuitPython_RGBLED
