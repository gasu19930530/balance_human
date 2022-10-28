# -*- coding: utf-8 -*-

import smbus
import math
from time import sleep
import time
import numpy as np
from ahrs.filters import Madgwick
from pyquaternion import Quaternion

import sys
import random
import RPi.GPIO as GPIO
import wiringpi as pi

MOTOR1_CW = 23
MOTOR1_CCW = 24
MOTOR2_CW = 25
MOTOR2_CCW = 26
OUTPUT_PIN_1 = 21
OUTPUT_PIN_2 = 20
GPIO.setmode( GPIO.BCM )
GPIO.setup( MOTOR1_CW, GPIO.OUT )
GPIO.setup( MOTOR1_CCW, GPIO.OUT )
GPIO.setup( MOTOR2_CW, GPIO.OUT )
GPIO.setup( MOTOR2_CCW, GPIO.OUT )
pi.wiringPiSetupGpio()
pi.pinMode(OUTPUT_PIN_1, pi.OUTPUT)
pi.pinMode(OUTPUT_PIN_2, pi.OUTPUT)
pi.softPwmCreate(OUTPUT_PIN_1, 0, 100)
pi.softPwmCreate(OUTPUT_PIN_2, 0, 100)
print("・・・・GPIO setted")

M = 0.00        #与える操作量
goal = 0.00    #目的値
e = 0.00        #偏差(目的値と現在値の差)
e1 = 0.00       #前回の偏差
e2 = 0.00       #前々回の偏差
Kp = float(sys.argv[1])       #比例制御（P制御)の比例定数
Ki = float(sys.argv[2])        #積分制御（I制御)の比例定数
Kd = float(sys.argv[3])        #微分制御（D制御)の比例定数
print("Kp=%.2f Ki=%.2f Kd=%.2f"% (Kp,Ki,Kd))

motor_speed = 0
Max_speed = int(sys.argv[4])
print("MAX motor speed=%d"% (Max_speed))

offset_deg = float(sys.argv[5])
print("roll degree offset"% (offset_deg))

pregx = 0
pregy = 0
pregz = 0
preax = 0
preay = 0
preaz = 0

degree_x = 0
degree_y = 0
degree_z = 0

preQ = [1,0,0,0]

dt = 0.01

DEV_ADDR = 0x68

ACCEL_XOUT = 0x3b
ACCEL_YOUT = 0x3d
ACCEL_ZOUT = 0x3f
TEMP_OUT = 0x41
GYRO_XOUT = 0x43
GYRO_YOUT = 0x45
GYRO_ZOUT = 0x47

PWR_MGMT_1 = 0x6b
PWR_MGMT_2 = 0x6c

bus = smbus.SMBus(1)
bus.write_byte_data(DEV_ADDR, PWR_MGMT_1, 0)

def stop_motor():
    GPIO.output( MOTOR1_CW, GPIO.LOW )
    GPIO.output( MOTOR1_CCW, GPIO.LOW )
    GPIO.output( MOTOR2_CW, GPIO.LOW )
    GPIO.output( MOTOR2_CCW, GPIO.LOW )
    pi.softPwmWrite(OUTPUT_PIN_1, 0)
    pi.softPwmWrite(OUTPUT_PIN_2, 0)
    time.sleep(0.05)

def PID(now_deg, e1, e2):
    e = goal - now_deg #偏差（e） = 目的値（goal） - 現在地
    pid = Kp * (e-e1) + Ki * e + Kd * ((e-e1) - (e1-e2))
    e1 = e
    e2 = e1
    return pid, e1, e2

def quaternion_to_euler_zyx(q):
    """
    クォータニオンをz-y-x系オイラー角に変換する。

    Parameters
    ----------
    q : Quaternion
        クォータニオン(pyquaternion形式)

    Returns
    -------
    np.array
        z-y-x系オイラー角
    """

    # roll : x軸回転
    sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3])
    cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # pitch : y軸回転
    sinp = 2 * (q[0] * q[2] - q[3] * q[1])
    if math.fabs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)

    # yaw : z軸回転
    siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2])
    cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3])
    yaw = math.atan2(siny_cosp, cosy_cosp)

    # オイラー角
    return np.array([math.degrees(roll),math.degrees(pitch),math.degrees(yaw)])

def read_word(adr):
    high = bus.read_byte_data(DEV_ADDR, adr)
    low = bus.read_byte_data(DEV_ADDR, adr+1)
    val = (high << 8) + low
    return val

def read_word_sensor(adr):
    val = read_word(adr)
    if (val >= 0x8000):  return -((65535 - val) + 1)
    else:  return val

def get_temp():
    temp = read_word_sensor(TEMP_OUT)
    x = temp / 340 + 36.53      # data sheet(register map)記載の計算式.
    return x

def getGyro():
    x = read_word_sensor(GYRO_XOUT)/ 131.0
    y = read_word_sensor(GYRO_YOUT)/ 131.0
    z = read_word_sensor(GYRO_ZOUT)/ 131.0
    return [x, y, z]


def getAccel():
    x = read_word_sensor(ACCEL_XOUT)/ 16384.0
    y= read_word_sensor(ACCEL_YOUT)/ 16384.0
    z= read_word_sensor(ACCEL_ZOUT)/ 16384.0
    return [x, y, z]



stop_motor()

try:
    while True:
        ax, ay, az = getAccel()
        gx, gy, gz = getGyro()

        # acc_data = np.array([ax,ay,az])
        # gyro_data = np.array([gx,gy,gz])
        acc_data = np.array([[preax,preay,preaz],[ax,ay,az]])
        gyro_data = np.array([[pregx,pregy,pregz],[gx,gy,gz]])

        madgwick = Madgwick(gyr=gyro_data, acc=acc_data)
        # degree_x += (pregx + gx) * dt / 2
        # degree_y += (pregy + gy) * dt / 2;
        # degree_z += (pregz + gz) * dt / 2;

        euler = quaternion_to_euler_zyx(madgwick.Q[0])
        roll_deg = euler[0] - offset_deg

        # print("\r%f,%f,%f,%f" % (madgwick.Q[0][0],madgwick.Q[0][1],madgwick.Q[0][2],madgwick.Q[0][3]),end="")
        motor_info = PID(roll_deg, e1, e2)
        motor_speed = motor_speed + motor_info[0] #操作量　→　モーター速度
        if motor_speed >= Max_speed:
            motor_speed = Max_speed
        elif motor_speed <= -Max_speed:
            motor_speed = -Max_speed
        e1 = motor_info[1]
        e2 = motor_info[2]

        print("\r SPEED %.1f 操作量　%.2f　角度　%.2f   " % (motor_speed,motor_info[0],roll_deg), end="")

        if (motor_speed >= 0):
            duty_rate = motor_speed
            GPIO.output( MOTOR1_CW, GPIO.LOW )
            GPIO.output( MOTOR1_CCW, GPIO.HIGH )
            GPIO.output( MOTOR2_CW, GPIO.LOW )
            GPIO.output( MOTOR2_CCW, GPIO.HIGH )
        elif (motor_speed < 0):
            duty_rate = motor_speed * -1
            GPIO.output( MOTOR1_CW, GPIO.HIGH )
            GPIO.output( MOTOR1_CCW, GPIO.LOW )
            GPIO.output( MOTOR2_CW, GPIO.HIGH )
            GPIO.output( MOTOR2_CCW, GPIO.LOW )

        pi.softPwmWrite(OUTPUT_PIN_1, int(duty_rate))
        pi.softPwmWrite(OUTPUT_PIN_2, int(duty_rate))

        time.sleep(dt)
        pregx = gx
        pregy = gy
        pregz = gz
        preax = ax
        preay = ay
        preaz = az

except KeyboardInterrupt:
    GPIO.cleanup()
    print("GPIO cleaned")
