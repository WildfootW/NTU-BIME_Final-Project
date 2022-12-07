import RPi.GPIO as GPIO
import time

#GPIO Mode (BOARD / BCM)
GPIO.setmode(GPIO.BOARD)

#set GPIO Pins
PWM1 = 37

#set GPIO direction (IN / OUT)
GPIO.setup(PWM1, GPIO.OUT)

if __name__ == '__main__':
    try:
        seconds = 1
        while True:
            print("Rotate")
            time.sleep(seconds)
            GPIO.output(PWM1, False)

        # Reset by pressing CTRL + C
    except KeyboardInterrupt:
        print("Stopped by User")
        GPIO.cleanup()
