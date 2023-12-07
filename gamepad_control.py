import os
import pygame

from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

# RaspberryPi上で動かす場合、必要になる
os.environ["SDL_VIDEODRIVER"] = "dummy"
pygame.init()
joystick = pygame.joystick.Joystick(0)
joystick.init()

# PCA9685
i2c = busio.I2C(SCL, SDA)
pca1 = PCA9685(i2c, address=0x40)
pca1.frequency = 50
pca2 = PCA9685(i2c, address=0x41)
pca2.frequency = 50

TYPE_A = 0
TYPE_B = 6
TYPE_C = 12

servos = []

# 線形補間をするための関数を返す
def make_interpolater(left_min, left_max, right_min, right_max): 
    left_span = left_max - left_min  
    right_span = right_max - right_min  
    scale_factor = float(right_span) / float(left_span) 
    def tmp(value):
        return right_min + (value-left_min) * scale_factor
    return tmp

if __name__ == '__main__':

    for i in range(6):
        tmp = servo.Servo(pca1.channels[i],
                min_pulse=500, max_pulse=2500, actuation_range=270)
        servos.append(tmp)
    for i in range(6):
        tmp = servo.Servo(pca1.channels[6+i],
                min_pulse=500, max_pulse=2500, actuation_range=180)
        servos.append(tmp)
    for i in range(6):
        tmp = servo.Servo(pca2.channels[i],
                min_pulse=500, max_pulse=2500, actuation_range=180)
        servos.append(tmp)

    # 操作する関節番号
    target1 = 0
    # 操作する関節種類の指定 (3種類)
    target2 = 0

    while True:
        # GamePadからの入力を取得
        if pygame.event.get():
            gamepad_data = {
                "joy_lx": joystick.get_axis(0),
                "joy_ly": joystick.get_axis(1),
                "joy_ry": joystick.get_axis(2),
                "joy_rx": joystick.get_axis(3),
                "hat_x": joystick.get_hat(0)[0],
                "hat_y": joystick.get_hat(0)[1],
                "btn_a": joystick.get_button(0),
                "btn_b": joystick.get_button(1),
                "btn_x": joystick.get_button(2),
                "btn_y": joystick.get_button(3),
                "btn_lb": joystick.get_button(4),
                "btn_rb": joystick.get_button(5),
                "btn_back": joystick.get_button(6),
                "btn_start": joystick.get_button(7),
                "btn_guide": joystick.get_button(8),
                "btn_joyl": joystick.get_button(9),
                "btn_joyr": joystick.get_button(10)
            }
            #print(gamepad_data)

            value = gamepad_data['joy_lx']
            if target2 == 0:
                # 150度の±90度で動かす
                f = make_interpolater(-1.0,+1.0, 150-90, 150+90)
                angle = f(value)
                servos[TYPE_A + target1].angle = angle
                print("TYPE_A=", angle)
            if target2 == 1:
                # 140度の±30度で動かす
                f = make_interpolater(-1.0,+1.0, 140-30, 140+30)
                angle = f(value)
                servos[TYPE_B + target1].angle = angle
                print("TYPE_B=", angle)
            if target2 == 2:
                # 90度の±60度で動かす
                f = make_interpolater(-1.0,+1.0, 90-60, 90+60)
                angle = f(value)
                servos[TYPE_C + target1].angle = angle
                print("TYPE_C=", angle)

            if gamepad_data['btn_x']:
                target1 += 1
                if target1 == 6:
                    target1 = 0
            if gamepad_data['btn_y']:
                target2 += 1
                if target2 == 3:
                    target2 = 0