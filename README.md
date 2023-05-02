# CAN-bus gauge cluster for BMW E12
![alt text](https://github.com/pazi88/E12_CAN_gauges/tree/main/Pics/20230214_115921.jpg?raw=true)

This is repository for CAN-bus instrument cluster for BMW E12. This is meant to replace the stock 70's mechanics/electronics inside the
cluster with x27 steppers and OLED screens to make the cluster to work with modern ECUs through CAN-bus, but retain original look.
Even though this project is meant only for E12 BMW, the code and basic circuit design can be used for other cars too.

X27.589 steppers are used to move the needles and two black and white SH1122 OLED screens are used to display odometer and trip.
MCU controlling the cluster is STM32F103C8T6 "blue pill" because of the cheap price, good availability and integrated CAN-interface.
For CAN-interface I'm using STM32 CAN library written by me: https://github.com/pazi88/STM32_CAN

CAN-bus currently supports reading data in BMW e46/e39 CAN-bus format (can11h) and also in OBD2 format.

The OLED screen used: https://s.click.aliexpress.com/e/_DnFnmSD
The steppers screen used: https://s.click.aliexpress.com/e/_DCNMbOH
STM32F103C8T6: https://s.click.aliexpress.com/e/_DE6f1d3


## EasyEda
EasyEda/OSHWLab project link for cluster back board: https://oshwlab.com/pazi88/e12_mittaristo
EasyEda/OSHWLab project link for needle back board: https://oshwlab.com/pazi88/e12_mittaristo_copy