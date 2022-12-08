# SPDX-FileCopyrightText: 2017 Limor Fried for Adafruit Industries
#
# SPDX-License-Identifier: MIT
# python3 -m pip install adafruit-circuitpython-dht
# https://github.com/adafruit/Adafruit_CircuitPython_DHT
# https://docs.circuitpython.org/projects/dht/en/3.0.1/

import time

import adafruit_dht
import board

dht = adafruit_dht.DHT11(board.D19)

while True:
    try:
        temperature = dht.temperature
        humidity = dht.humidity
        # Print what we got to the REPL
        if temperature and humidity:
            print("Temp: {%r} *C \t Humidity: {%r}" % (temperature, humidity))
    except RuntimeError as e:
        # Reading doesn't always work! Just print error and we'll try again
        print("Reading from DHT failure: ", e.args)

    time.sleep(1)
