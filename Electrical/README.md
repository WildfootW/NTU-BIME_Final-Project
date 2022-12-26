# BIME Final Project - Electrical
[TOC]
###### tags: `Mechatronics`


# BIME Final ~~HAT~~ PCB
* Reference Diameter
    * ![](https://i.imgur.com/v4VtRLp.png)
![](https://i.imgur.com/yKnSz3C.jpg)
 
 ![](https://i.imgur.com/yYgB5HN.png)

* [How to Export Gerber](https://www.elecrow.com/wiki/index.php?title=How_to_export_gerber_files_from_Altium_designer)

## Todo
* add Pinout
* Add 6V Switch
* Correct Fan Position
* repoly first layer
* reverse 10x2 pin
* Correct Resistor Ohm (R6789)
![](https://i.imgur.com/l5kfLvD.png)
https://roboticsbackend.com/raspberry-pi-3-pins/#Reserved_pins
[**raspberrypi / hats**](https://github.com/raspberrypi/hats)
* move buzzer to pin7
* move Fan_Tach to pin26

## 3V3 to 5V
### Unidirectional
* [Amplify PWM Signal from exactly 0-3.3V to 0-5V](https://electronics.stackexchange.com/questions/359994/amplify-pwm-signal-from-exactly-0-3-3v-to-0-5v)
* Gate
    * ![](https://i.imgur.com/9kPJOH9.png)
* BJT
    * ![](https://i.imgur.com/4L9G70u.png)
* [Auto fan control trouble shooting](https://forums.raspberrypi.com/viewtopic.php?t=328330)
    * ![](https://i.imgur.com/YZeQ5H4.png)
![](https://i.imgur.com/4Iy7j0a.png)

### Bidirectional OC/OD open-collector or open-drain
* Buffer
    * [ I2C buffers overview ](https://youtu.be/WZWSB2-kvuA)
    * ![](https://i.imgur.com/1nZsvv4.png)
* Translator
    * [ I2C translators overview ](https://youtu.be/_Rxt1LLvtf4)
    * ![](https://i.imgur.com/5hvUME6.png)
    
* Compiled Tips ‘N Tricks Guide
![](https://i.imgur.com/3t6xLkt.png)

    
# Buck Converter
* [EasyEDA Project](https://u.easyeda.com/account/user/projects/index/detail?project=bbbc860793c24aa5b11dd05bab18da10&folder=all)

## Manufacture
* JLCPCBA
* [【教學】最簡單的PCB電路板教學，從設計到廠商製造一次搞定。JLCPCB](https://youtu.be/yPJ-4nEXykE)
* [製作自己的第一片PCB—使用JLCPCB、EasyEDA](https://bo-sgoldhouse.blogspot.com/2021/08/pcb-jlcpcb-easyeda.html)
* ![](https://i.imgur.com/DsbFzOl.png)
* ![](https://i.imgur.com/1nmL6qf.png)

## RPi 3B+ Power
* [Cable](https://shopee.tw/-%E7%89%A9%E8%81%AF%E6%B1%AA%E6%B1%AA-%E5%AF%A6%E7%94%A8-Micro-USB-%E9%96%8B%E9%97%9C%E5%85%85%E9%9B%BB%E7%B7%9A%E9%9B%BB%E6%BA%90%E7%B7%9A%E9%81%A9-Raspberry-Pi-3B-RPi%E6%A8%B9%E8%8E%93%E6%B4%BE-Android-i.9340697.59613649?sp_atk=c7cfdbeb-d320-4970-9cd8-a7ded9f09e9f&xptdk=c7cfdbeb-d320-4970-9cd8-a7ded9f09e9f)
* [What is the expected behaviour of the PWR LED on Pi3B+](https://forums.raspberrypi.com/viewtopic.php?t=212350)
* [RPi 3 B: Strange behaviour of power LED](https://forums.raspberrypi.com/viewtopic.php?t=147447)
* [What do system LEDs signify?](https://raspberrypi.stackexchange.com/questions/871/what-do-system-leds-signify/60563#60563)
* [Raspberry Pi 3 Model B+ Stable Red Light When Powered Up](https://raspberrypi.stackexchange.com/questions/83090/raspberry-pi-3-model-b-stable-red-light-when-powered-up)
* ![](https://i.imgur.com/5T8AnSV.png)
* ![](https://i.imgur.com/KCNThUR.png)



## Circuit
![](https://i.imgur.com/7xtZ6ZX.png)
![](https://i.imgur.com/qBkX1xw.png)

### MP2315
[How to Design 12V to 5V Buck Converter 一起设计一款好用的12V转5V电源模块](https://www.youtube.com/playlist?list=PLBpCr1fi_kFbyh0TMSjQk9jeb3ThH9svX)

#### Schematic
![](https://i.imgur.com/5bnPacg.png)

![](https://i.imgur.com/DHoqf00.png)

#### Layout
* Ver.傳奇帝 
	* https://oshwhub.com/jie.li/dcdc_mp2315
	* ![](https://i.imgur.com/hcldyah.png)
	* ![](https://i.imgur.com/QzYANjy.png)
	* ![](https://i.imgur.com/a8yndK6.png)

* Ver.孫
	* https://oshwhub.com/sunhaoqin/dcdc_mp2315
	* ![](https://i.imgur.com/BXy1aTk.png)
	* ![](https://i.imgur.com/nxwX3g1.jpg)
    
#### Test
* No Loading
    * ![](https://i.imgur.com/KAFuGTf.jpg)
* RPi Power on
    * ![](https://i.imgur.com/bQi5wwc.jpg)
* RPi not stable while operation
    * ![](https://i.imgur.com/DZ9tpBU.jpg)
    * The PWR LED will turn off if the current of 12V over 0.3A. The RPi did not reset. The oscilloscope can neither find the 5V drop.
    * Replace the module by LM-2596. The PWR LED become more stable with less LED off. The current is less.
    * Connect the USB A direct to 5V Power supply. The PWR LED is stable as LM-2596. (not for sure). And current below 0.2A.
    * Connect with my power bank. RPi is not stable as well.
    * Connect direct to AC Converter. The PWR LED is very stable.

### LM-2596
![](https://i.imgur.com/3GgDa3e.png)
* [ LM2596S 降壓電源 DC-DC 降壓型 電源模組  具備電源指示燈 ](https://www.taiwaniot.com.tw/product/lm2596s-%E9%99%8D%E5%A3%93%E9%9B%BB%E6%BA%90-dc-dc-%E9%99%8D%E5%A3%93%E5%9E%8B-%E9%9B%BB%E6%BA%90%E6%A8%A1%E7%B5%84-%E5%8F%AF%E8%AA%BF%E9%99%8D%E5%A3%93%E6%A8%A1%E7%B5%84/)

#### Module
* Fake LM2596 & Detail Test
* [ Counterfeit LM2596 Regulator Boards ](https://k6jca.blogspot.com/2018/02/counterfeit-lm2596-regulator-boards.html)
* ![](https://i.imgur.com/6OnMKeU.png)


