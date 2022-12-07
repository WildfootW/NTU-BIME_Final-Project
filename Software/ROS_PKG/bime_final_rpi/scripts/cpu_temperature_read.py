#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyleft (ɔ) 2022 wildfootw <wildfootw@wildfoo.tw>
#
# Distributed under terms of the MIT license.

def cpu_temperature():
    temperature_file = open('/sys/class/thermal/thermal_zone0/temp')
    temperature_c = float(temperature_file.read()) / 1000
    temperature_file.close()
    return temperature_c

if __name__ == "__main__":
    while True:
        print(cpu_temperature())
