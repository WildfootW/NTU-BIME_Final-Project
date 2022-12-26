# BIME Final Project - Components
[TOC]
###### tags: `Mechatronics`

# Software & Circuit
## Reference
* [Duckiebot](https://docs.duckietown.org/daffy/opmanual_duckiebot/out/duckiebot_configurations.html)
* [ nasa-jpl / open-source-rover ](https://github.com/nasa-jpl/open-source-rover)
* ROS Ecosystem
* [t1m0thyj / picar](https://github.com/t1m0thyj/picar)
    * [ How to Make Raspberry Pi Car at home ](https://youtu.be/FyMq5KOcgTg)

## Communication
### Video Transmitter
[ 5.8GHz Vs. 1300MHz FPV Penetrating Power Test - RCTESTFLIGHT - ](https://youtu.be/LltDtKs2sQ4)

### FPV Transmitter / Receiver
* [Drones and Model Aircraft - Frequency](https://drones.stackexchange.com/questions/810/why-is-5-8ghz-used-for-fpv-and-2-4ghz-use-for-transmitters)
![](https://i.imgur.com/DXZW0zg.png)

### On Screen Display (OSD)
* [Raspberry Pi Zero FPV camera and OSD](https://hackaday.io/project/12450-raspberry-pi-zero-fpv-camera-and-osd)

### Direction

## Power
* Battery
* BMS
![](https://i.imgur.com/3pmH4Qd.png)
RPi input from USB Header 5V
![](https://i.imgur.com/EZXqdNk.png)
http://www.pidramble.com/wiki/benchmarks/power-consumption

## Sensor
### Camera
* C310 HD
* Intel® RealSense™ depth camera D435

### Obstacle
* Depth Camera
* Sonic
* IR

### Thermal
https://lastminuteengineers.com/mlx90614-ir-temperature-sensor-arduino-tutorial/

測量熱源的關卡設定是在熱源前面35-45公分處（助教指定距離），定位後車輛停止五秒感測溫度，目前實際測試看來，溫度在攝氏40-45度之間。
![](https://i.imgur.com/oJZC3KL.png)
![](https://i.imgur.com/fMWGlO1.png)


* Fan

### Accelerometer?
MPU-6050
![](https://i.imgur.com/TTqln54.png)
![](https://i.imgur.com/iH6ToTg.png)
https://shop.cpu.com.tw/product/45284/info/

## Actuator
### PWM
* [Adafruit 16-Channel 12-bit PWM/Servo Driver - I2C interface - PCA9685](https://www.adafruit.com/product/815)
* [What's the difference between soft PWM and PWM](https://raspberrypi.stackexchange.com/questions/100641/whats-the-difference-between-soft-pwm-and-pwm)
* [ besp9510 /dma_pwm ](https://github.com/besp9510/dma_pwm)
### H-Bridge
![](https://i.imgur.com/xFWR53z.png)

### Wheel Motor
* [Adafruit TT](https://www.adafruit.com/product/3777)
* ![](https://i.imgur.com/IRJdvfx.png)
* ![](https://i.imgur.com/KWdhHlc.png)

### Gripper
* [Tower Pro SG90](https://www.towerpro.com.tw/product/sg90-7/)
* ![](https://i.imgur.com/GbqmRK1.png)

### Buzzer


# Mechanical
## References
* [A gamified simulator and physical platform for self-driving algorithm training and validation](https://www.researchgate.net/figure/This-1-10th-scale-buggy-features-a-Raspberry-Pi-3B-Navio2-interface-board-Logitech_fig10_337336443)
    * ![](https://i.imgur.com/ueaEHE7.png)
* [ nasa-jpl /open-source-rover ](https://github.com/nasa-jpl/open-source-rover)

## Car Body
* 4x Mecanum wheel W/O Suspenstion
* 4x Mecanum wheel W/ Suspenstion
    * https://youtu.be/K1yDDmkbaq4
* 3x Omni Wheel
* 4x Normal Wheel W/ Suspension
    * HSP94186 (https://youtu.be/iLiI_IRedhI)
    * [Donkey® Car](https://docs.donkeycar.com)
* 4x Normal Wheel W/O Suspension
* 2x Normal Wheel W/ Ball Roller
    * [Li-Wei Yang - Mechatronics 4](https://youtu.be/lFlVofkHzSU)

## Gripper
* 球的直徑大約為6.6-6.7公分

## Holders
* Camera
* Thermal
* ...

