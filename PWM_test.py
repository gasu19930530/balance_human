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

MOTOR_CW = 23
MOTOR_CCW = 24
# MOTOR_STBY = 25
# MOTOR_R2 = 25

OUTPUT_PIN_1 = 21
# OUTPUT_PIN_R = 11

GPIO.setmode( GPIO.BCM )
GPIO.setup( MOTOR_CW, GPIO.OUT )
GPIO.setup( MOTOR_CCW, GPIO.OUT )
# GPIO.setup( MOTOR_STBY, GPIO.OUT )
# GPIO.setup( MOTOR_R2, GPIO.OUT )

pi.wiringPiSetupGpio()
pi.pinMode(OUTPUT_PIN_1, pi.OUTPUT)
# pi.pinMode(OUTPUT_PIN_R, pi.OUTPUT)

pi.softPwmCreate(OUTPUT_PIN_1, 0, 100)
#pi.softPwmCreate(OUTPUT_PIN_R, 0, 100)

def stop_motor():
    GPIO.output( MOTOR_CW, GPIO.LOW )
    GPIO.output( MOTOR_CCW, GPIO.LOW )
    # GPIO.output( MOTOR_STBY, GPIO.HIGH )
    # GPIO.output( MOTOR_R2, GPIO.LOW )

    pi.softPwmWrite(OUTPUT_PIN_1, 0)
    # pi.softPwmWrite(OUTPUT_PIN_R, 0)

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
        GPIO.output( MOTOR_CW, GPIO.HIGH )
        GPIO.output( MOTOR_CCW, GPIO.LOW )
        pi.softPwmWrite(OUTPUT_PIN_1, 10)

    elif( a == "2" ):
        print("Foward,15")
        GPIO.output( MOTOR_CW, GPIO.HIGH )
        GPIO.output( MOTOR_CCW, GPIO.LOW )
        pi.softPwmWrite(OUTPUT_PIN_1, 15)

    elif( a == "3" ):
        print("Foward,20")
        GPIO.output( MOTOR_CW, GPIO.HIGH )
        GPIO.output( MOTOR_CCW, GPIO.LOW )
        pi.softPwmWrite(OUTPUT_PIN_1, 20)

    elif( a == "4" ):
        print("Foward,25")
        GPIO.output( MOTOR_CW, GPIO.HIGH )
        GPIO.output( MOTOR_CCW, GPIO.LOW )
        pi.softPwmWrite(OUTPUT_PIN_1, 25)

    elif( a == "5" ):
        print("Foward,30")
        GPIO.output( MOTOR_CW, GPIO.HIGH )
        GPIO.output( MOTOR_CCW, GPIO.LOW )
        pi.softPwmWrite(OUTPUT_PIN_1, 30)

    elif( a == "11" ):
        print("Return,10")
        GPIO.output( MOTOR_CW, GPIO.LOW )
        GPIO.output( MOTOR_CCW, GPIO.HIGH )
        pi.softPwmWrite(OUTPUT_PIN_1, 10)

    elif( a == "12" ):
        print("Return,15")
        GPIO.output( MOTOR_CW, GPIO.LOW )
        GPIO.output( MOTOR_CCW, GPIO.HIGH )
        pi.softPwmWrite(OUTPUT_PIN_1, 15)

    elif( a == "13" ):
        print("Return,20")
        GPIO.output( MOTOR_CW, GPIO.LOW )
        GPIO.output( MOTOR_CCW, GPIO.HIGH )
        pi.softPwmWrite(OUTPUT_PIN_1, 20)

    elif( a == "14" ):
        print("Return,25")
        GPIO.output( MOTOR_CW, GPIO.LOW )
        GPIO.output( MOTOR_CCW, GPIO.HIGH )
        pi.softPwmWrite(OUTPUT_PIN_1, 25)

    elif( a == "15" ):
        print("Return,30")
        GPIO.output( MOTOR_CW, GPIO.LOW )
        GPIO.output( MOTOR_CCW, GPIO.HIGH )
        pi.softPwmWrite(OUTPUT_PIN_1, 30)

    elif( a == "20" ):
        print("Stop")
        stop_motor()

    else:
        print("THE END")
        GPIO.cleanup()
        break
