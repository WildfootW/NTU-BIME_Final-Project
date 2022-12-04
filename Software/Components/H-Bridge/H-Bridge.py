import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
GPIO_1_1 = 7
GPIO_1_2 = 11

#set GPIO direction (IN / OUT)
GPIO.setup(GPIO_1_1, GPIO.OUT)
GPIO.setup(GPIO_1_2, GPIO.OUT)

def forward(sec):
    print("forward")
    GPIO.output(GPIO_1_1, False)
    GPIO.output(GPIO_1_2, True)

def backward(sec):
    print("backward")
    GPIO.output(GPIO_1_1, True)
    GPIO.output(GPIO_1_2, False)

if __name__ == '__main__':
    try:
        seconds = 1
        while True:
            time.sleep(seconds)
            forward(seconds)
            time.sleep(seconds)
            backward(seconds)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Measurement stopped by User")
        GPIO.cleanup()
