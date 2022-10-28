#!/usr/bin/python
# coding: UTF-8

import sys
import time
# import bluetooth
import random
import RPi.GPIO as GPIO
import wiringpi as pi

# address = "B8:27:EB:F9:EF:95"
# port = 1

MOTOR_LARGE = 23
MOTOR_MODE = 24
MOTOR_STBY = 25
# MOTOR_R2 = 25

OUTPUT_PIN_F = 21
OUTPUT_PIN_R = 11

GPIO.setmode( GPIO.BCM )
GPIO.setup( MOTOR_LARGE, GPIO.OUT )
GPIO.setup( MOTOR_MODE, GPIO.OUT )
GPIO.setup( MOTOR_STBY, GPIO.OUT )
# GPIO.setup( MOTOR_R2, GPIO.OUT )

pi.wiringPiSetupGpio()
pi.pinMode(OUTPUT_PIN_F, pi.OUTPUT)
pi.pinMode(OUTPUT_PIN_R, pi.OUTPUT)

pi.softPwmCreate(OUTPUT_PIN_F, 0, 100)
pi.softPwmCreate(OUTPUT_PIN_R, 0, 100)

def stop_motor():
    GPIO.output( MOTOR_LARGE, GPIO.HIGH )
    GPIO.output( MOTOR_MODE, GPIO.LOW )
    GPIO.output( MOTOR_STBY, GPIO.HIGH )
    # GPIO.output( MOTOR_R2, GPIO.LOW )

    pi.softPwmWrite(OUTPUT_PIN_F, 0)
    pi.softPwmWrite(OUTPUT_PIN_R, 0)

    time.sleep(0.05)

stop_motor()

# sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )

# sock.bind( ( "", port ) )
# sock.listen( 1 )

# while ( True ):
#     try:
#         print ("Connect....")
#         sock_client,address = sock.accept()
#         print("Connected")
#         break
#     except bluetooth.BluetoothError :
#         print ("Connection Failed")
#         sock = bluetooth.BluetoothSocket( bluetooth.RFCOMM )
#         time.sleep ( 1 )

while 1:
    # data_ = sock_client.recv(16)
    # data  = data_.decode("UTF-8")

    # for i in data:
    a = input('数値を入力>>')
    if( a == "1" ):
        print("Foward,10")
        pi.softPwmWrite(OUTPUT_PIN_F, 10)
        pi.softPwmWrite(OUTPUT_PIN_R, 0)

    elif( a == "2" ):
        print("Foward,15")
        pi.softPwmWrite(OUTPUT_PIN_F, 15)
        pi.softPwmWrite(OUTPUT_PIN_R, 0)

    elif( a == "3" ):
        print("Foward,20")
        pi.softPwmWrite(OUTPUT_PIN_F, 20)
        pi.softPwmWrite(OUTPUT_PIN_R, 0)

    elif( a == "4" ):
        print("Foward,25")
        pi.softPwmWrite(OUTPUT_PIN_F, 25)
        pi.softPwmWrite(OUTPUT_PIN_R, 0)

    elif( a == "5" ):
        print("Foward,30")
        pi.softPwmWrite(OUTPUT_PIN_F, 30)
        pi.softPwmWrite(OUTPUT_PIN_R, 0)

    elif( a == "11" ):
        print("Return,10")
        pi.softPwmWrite(OUTPUT_PIN_F, 0)
        pi.softPwmWrite(OUTPUT_PIN_R, 10)

    elif( a == "12" ):
        print("Return,15")
        pi.softPwmWrite(OUTPUT_PIN_F, 0)
        pi.softPwmWrite(OUTPUT_PIN_R, 15)

    elif( a == "13" ):
        print("Return,20")
        pi.softPwmWrite(OUTPUT_PIN_F, 0)
        pi.softPwmWrite(OUTPUT_PIN_R, 20)

    elif( a == "14" ):
        print("Return,25")
        pi.softPwmWrite(OUTPUT_PIN_F, 0)
        pi.softPwmWrite(OUTPUT_PIN_R, 25)

    elif( a == "15" ):
        print("Return,30")
        pi.softPwmWrite(OUTPUT_PIN_F, 0)
        pi.softPwmWrite(OUTPUT_PIN_R, 30)

    elif( a == "20" ):
        print("Stop")
        stop_motor()

    else:
        print("THE END")
        GPIO.cleanup()
        break
