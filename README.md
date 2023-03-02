# "BigSpot"

demo video : https://youtu.be/u6ZRoQI9YBw

The BigSpot has the following main components:

## Hardware
- Raspberry Pi 4B + 
- Raspberry Pi Camera

- MicroSD 32G 
- 4 x 12A Step Down Module
- 2 × HC-SR04P Ultrasonic sensor
- 1 × MPU-6050 Gyro sensor
- 1 × I2C 16x2 LCD Module
- 7.4v Battery ( 2s2p )
- 6 x F625ZZ
- 2 x pca9685
- 12 x RDS5160 servo moter
- 20A Push button (Lock type)
- 18AWG
- 12V 5A Adaptor ( test )
- 50A LED Display
- PS4 DualShock compatible Joystick
- XT60H +/-

- USB wlan / NEXT-202N-MINI
- HDMI to MICRO HDMI gender
- IMAX B6
- UK 831N Multi tester

## gazebo
```
roslaunch bigspot run_gazebo_only.launch
roslaunch bigspot_joystick ramped_keyboard.launch
```

## real
```
roslaunch bigspot run_robot_real.launch
roslaunch bigspot_description controller.launch
roslaunch bigspot_joystick ramped_joystick.launch
```

## etc
```
roslaunch bigspot_mpu6050 mpu.launch
roslaunch bigspot_mpu9250 mpu.launch
roslaunch bigspot_lcd lcd.launch
roslaunch bigspot_rgb rgb.launch
roslaunch bigspot_ultrasonic ultrasonic.launch
```

## packages
```
sudo apt install ros-noetic-joy
sudo apt install ros-noetic-ros-controllers
sudo apt install ros-noetic-imu-tools
sudo apt install libgpiod2

pip3 install pygame
pip3 install adafruit-circuitpython-rgbled
pip3 install adafruit-circuitpython-motor
pip3 install adafruit-circuitpython-servokit
pip3 install adafruit-circuitpython-rgbled 	# https://github.com/adafruit/Adafruit_CircuitPython_RGBLED
```


## Credits
 - mike4192  : https://github.com/mike4192/spotMicro
 - lnotspotl : https://github.com/lnotspotl/notspot_sim_py
