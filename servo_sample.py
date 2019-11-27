#!/usr/bin/env python
# -*- coding: utf-8 -*-
import RPi.GPIO as GPIO
import time
from gpiozero import AngularServo
GPIO.setmode(GPIO.BCM)
#SG92Rをコントロールするための
class SG90_92R_Class:
    # mPin : GPIO Number (PWM)
    # mPwm : Pwmコントロール用のインスタンス
    # m_Zero_offset_duty :

    """コンストラクタ"""
    def __init__(self, Pin, ZeroOffsetDuty):
        self.mPin = Pin
        self.m_ZeroOffsetDuty = ZeroOffsetDuty

        #GPIOをPWMモードにする
        GPIO.setup(self.mPin, GPIO.OUT)
        self.mPwm = GPIO.PWM(self.mPin , 50) # set 20ms / 50 Hz

    """位置セット"""
    def SetPos(self,pos):
        #Duty ratio = 2.5%〜12.0% : 0.5ms〜2.4ms : 0 ～ 180deg
        duty = (12-2.5)/180*pos+2.5 + self.m_ZeroOffsetDuty
        self.mPwm.start(duty)


    """終了処理"""
    def Cleanup(self):
        #サーボを10degにセットしてから、インプットモードにしておく
        self.SetPos(0)
        time.sleep(1)
        GPIO.setup(self.mPin, GPIO.IN)

"""コントロール例"""
'''if __name__ == '__main__':
    #Useing GPIO No.  to idetify channel
    GPIO.setmode(GPIO.BCM)
    Servo = SG90_92R_Class(Pin=4,ZeroOffsetDuty=0)
    try:
        while True:
            Servo.SetPos(0)
            time.sleep(0.5)
            Servo.SetPos(45)
            time.sleep(0.5)
            Servo.SetPos(90)
            time.sleep(0.5)
            Servo.SetPos(0)
            time.sleep(0.5)
    except KeyboardInterrupt  :         #Ctl+Cが押されたらループを終了
        print("\nCtl+C")
    except Exception as e:
        print(str(e))
    finally:
        #Servo.Cleanup()
        #GPIO.cleanup()
        print("\nexit program")'''
GPIO.setmode(GPIO.BCM)
#Servo = SG90_92R_Class(Pin=4,ZeroOffsetDuty=0)
#Servo.SetPos(0)
#time.sleep(0.5)
#Servo.SetPos(45)
#time.sleep(0.5)
#Servo.SetPos(90)
#time.sleep(0.5)
#Servo.SetPos(0)
#time.sleep(0.5)

pig = AngularServo(4, min_pulse_width=0.5/1000, max_pulse_width=2.4/1000)
pig.angle = 90
time.sleep(1)

pig.angle = 45
time.sleep(1)

pig.angle = 0
time.sleep(1)
