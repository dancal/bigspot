# "BigSpot" - quadruped robot

## Fusion / Isaac ROS 5.0 simulation work

The 2026-10-10 [whole-motion audit](simulation/QUALITY.ko.md) removes coarse
angle stair steps across body routines, matches horizontal foot velocity at
swing boundaries, confirms lifted-paw landing before weight return, and waits
for sensor settling before allowing another action after walking stops.
The catalog contains 30 routines; jumping and vertical acrobatics remain
unvalidated experiments. These results use local ROS 2 Humble / MuJoCo CPU.
Real hardware, full self-collision and Isaac GPU execution are unverified.

The current default dance is `dance_funk`: 120 BPM, diagonal pair steps
every 0.5 seconds, with hip wiggles, front-body shakes and nods, side shuffles,
forward/back steps, turns and simultaneous body rhythms. There is no neck
joint; front emphasis is produced by the trunk and legs. See
[the current hip/front-body dance](simulation/GROOVE.ko.md).
The preceding hip/front-body update passed nine ROS cases, 95 code tests and
the local ROS package build. Its
37-second video includes the original beat and two fixed views at normal speed.
The preceding five-phrase version passed nine ROS cases and 91 code tests.
Its 25-second recording includes an original synthesized beat at normal speed.
Button 5 + 12 requests it; the selector contains 30 routines. See
[the preceding dance and measured validation](simulation/FUNK.ko.md).
The following paragraphs describe earlier development stages and their mappings.

See [the Korean setup and validation record](simulation/README.ko.md) for the new
ROS 2 Lyrical simulation adapter, Isaac Sim 6.0.1 bridge, Fusion audit scripts,
and original controller fixes. A local ROS 2 Humble / MuJoCo CPU simulation has
run the crouch, rise, balance, crawl and input-loss stop sequence. Thirty CEM
trials learned controller parameters, followed by independent validation and a
40-second actual ROS run. A gait review then corrected the rear stance, added
one-foot wave steps and landed-foot stopping, and repeated 30 trials with foot
clearance and body-motion metrics. The revised policy ran through real ROS for
24 seconds. A separate diagonal-trot profile then reached a measured
0.103 m/s, 10.4 times the slow profile, with a 24-second actual ROS run and a
60-second physics endurance check. The multi-action profile adds lateral/reverse
motion, sensor-controlled 90-degree turns, boost/run buttons and pose sequences,
with actual ROS validation using an RDS5160 x12, nominal 7.4V / 30A motor model.
Jump experiments did not meet flight criteria; rear standing violated current
joint workspace and the front standing attempt did not reach a vertical pose.
IMU SI conversion and sensor mounting configuration are implemented; real PWM
joint feedback and contact sensing remain unavailable. The adaptive profile adds
continuous acceleration/deceleration, jerk limits, dual forward-sonar braking,
relative heading hold and thirteen sensor-settled poses with support transfer.
Actual ROS trials cover sensor dropout, missing orientation, a lateral force
disturbance and mass/friction/noise holdouts. The optional GPIO sonar driver now
publishes typed Range messages in metres; physical acquisition remains untested.
Three grounded dance routines now use smooth entry/exit, tempo-based phrases,
IMU damping, measured joints and contact gates. Actual ROS tests completed the
60 BPM routines and checked stop/deadman buttons and sensor loss. A 90 BPM
mass/friction/noise trial finished but failed the three-foot support criterion;
tempo above 60 BPM in those routines requires simulation experiment opt-in. A larger routine now
has 5.6 times the measured height excursion and 3.8 times the roll excursion.
Four dog-like gestures add body tilts, an 8 cm front-paw wave and a play bow.
All five new nominal ROS actions completed. The mass/friction/noise holdout
passed the paw wave but failed the larger dance's strict support criterion.
A new 90 BPM rhythm uses a 2.67-second body cycle, three times the cadence of
the larger dance. Nominal and mass/friction/IMU-noise ROS trials both retained
at least three supporting feet. Joystick chords select and execute all 26
grounded routines, with edge triggering and stop/deadman/input-loss recovery.
Twelve new actual ROS cases and seventy-one regression tests pass. Button 12
selected the rhythm dance; physical gamepad mapping remains unverified.
A new larger 90 BPM routine now measures 69.9 mm of height excursion and
17.5 degrees of roll excursion, 2.38 and 2.97 times the preceding rhythm.
Seated clapping confirms three actual front-paw mesh contacts, then lands
and returns upright. Both routines pass nominal and mass/friction/noise ROS
trials. Fourteen new ROS cases and seventy-seven code tests pass. Button 8
requests seated clapping, button 12 requests the larger dance, and the
selector then contained 28 routines. See [the large dance and clap record](simulation/SHOW.ko.md).
The preceding paw-step dance transfers weight and lifts each paw, taps forward
or backward and outward, and confirms four landings. Measured clearance is
37--41 mm, with 35--45 mm fore/aft travel. Nominal and mass/friction/noise
holdouts retain at least three supporting paws. Nine actual ROS cases verify
completion, joystick operation, airborne cancellation and sensor-loss command
holds. Button 12 then selected paw steps; the selector had 29 routines, including
the preceding body dances. The 33.5-second recording shows two fixed views at
normal speed. See [the paw-step validation record](simulation/STEPS.ko.md).
GPU simulation,
physical hardware and mechanical CAD changes are not yet validated.

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
