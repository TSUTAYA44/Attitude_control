import pigpio
import time


pi = pigpio.pi()

class Servo_Class:

    """コンストラクタ"""
    def __init__(self, Pin, ZeroOffsetDuty):
        self.mPin = Pin
        self.m_ZeroOffsetDuty = ZeroOffsetDuty
        pi.set_mode(self.mPin, pigpio.ALT0)

    """位置セット"""
    def SetPos(self,degree):
        #Duty ratio = 2.5%〜12.0% : 0.5ms〜2.4ms : 0 ～ 180deg
        duty = int(((12 - 2.5) / 180 * pos + 2.5) * 10000)
        pi.hardware_PWM(self.mPin, 50, duty)

    """終了処理"""
    def Cleanup(self):
        #サーボを0degにセットしてから、インプットモードにしておく
        pi.hardware_PWM(self.mPin, 50, self.SetPos(0))
        time.sleep(1)
        pi.set_mode(self.mPin, pigpio.INPUT)
        pi.stop()