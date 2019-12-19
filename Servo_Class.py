import pigpio
import time


pi = pigpio.pi()
#SG92Rをコントロールするための
class Servo_Class:
    # mPin : GPIO Number (PWM)
    # mPwm : Pwmコントロール用のインスタンス
    # m_Zero_offset_duty :

    """コンストラクタ"""
    def __init__(self, Pin, ZeroOffsetDuty):
        self.mPin = Pin
        self.m_ZeroOffsetDuty = ZeroOffsetDuty
        pi.set_mode(self.mPin, pigpio.ALT0)
        #GPIO.setup(self.mPin, GPIO.OUT)
        #self.mPwm = GPIO.PWM(self.mPin , 50) # set 20ms / 50 Hz

    """位置セット"""
    def SetPos(self,degree):
        #Duty ratio = 2.5%〜12.0% : 0.5ms〜2.4ms : 0 ～ 180deg
        duty = (12-2.5)/180*degree+2.5 + self.m_ZeroOffsetDuty
        pi.hardware_PWM(self.mPin, 50, duty)
        #self.mPwm.start(duty)


    """終了処理"""
    def Cleanup(self):
        #サーボを0degにセットしてから、インプットモードにしておく
        pi.hardware_PWM(self.mPin, 50, self.SetPos(0))
        time.sleep(1)
        pi.set_mode(self.mPin, pigpio.INPUT)
        pi.stop()

servo = Servo_Class(18, 0)

servp.SetPos(degree=45)

servo.Cleanup()