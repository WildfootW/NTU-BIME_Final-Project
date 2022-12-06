import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
PWM2 = 11
PWM4 = 13
PWM6 = 15
PWM8 = 16
PWM3 = 33
PWM5 = 36
PWM7 = 31
PWM9 = 29

#set GPIO direction (IN / OUT)
GPIO.setup(PWM2, GPIO.OUT)
GPIO.setup(PWM4, GPIO.OUT)
GPIO.setup(PWM6, GPIO.OUT)
GPIO.setup(PWM8, GPIO.OUT)
GPIO.setup(PWM3, GPIO.OUT)
GPIO.setup(PWM5, GPIO.OUT)
GPIO.setup(PWM7, GPIO.OUT)
GPIO.setup(PWM9, GPIO.OUT)

def forward(sec):
    print("Forward")
    GPIO.output(PWM2, False)
    GPIO.output(PWM4, True)
    GPIO.output(PWM6, False)
    GPIO.output(PWM8, True)
    GPIO.output(PWM3, False)
    GPIO.output(PWM5, True)
    GPIO.output(PWM7, False)
    GPIO.output(PWM9, True)

def backward(sec):
    print("backward")
    GPIO.output(PWM2, True)
    GPIO.output(PWM4, False)

if __name__ == '__main__':
    try:
        seconds = 1
        while True:
#            time.sleep(seconds)
            forward(seconds)
            time.sleep(seconds)
#            backward(seconds)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Stopped by User")
        GPIO.cleanup()
