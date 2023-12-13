# coding: utf-8
import enum
import math
import os
import sys
import time
from distutils.util import strtobool

import numpy as np
from PyQt5 import QtWidgets
from PyQt5.QtCore import *
from PyQt5.QtGui import *
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import *

ENABLE_SERVO = strtobool(os.getenv("ENABLE_SERVO", "false")) == 1
USE_GAMEPAD = strtobool(os.getenv("USE_GAMEPAD", "false")) == 1

# I2C、PWMサーボモータードライバーの初期化 (実機のみ)
if ENABLE_SERVO:
    import busio
    from adafruit_motor import servo
    from adafruit_pca9685 import PCA9685
    from board import SCL, SDA
    os.environ["SDL_VIDEODRIVER"] = "dummy"

# Gamepadを使う場合
if USE_GAMEPAD:
    import pygame


class JointGroupId(enum.IntEnum):
    A = 0
    B = 1
    C = 2
    D = 3
    E = 4
    F = 5


class JointType(enum.IntEnum):
    J1 = 0
    J2 = 1
    J3 = 2
    J4 = 3

# よく使う計算ロジック（回転行列の計算）


class CalcUtil():
    @classmethod
    def rotation(cls, u, t):
        r = np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])
        return np.dot(r, u)

    @classmethod
    def make_interpolater(cls, left_min, left_max, right_min, right_max):
        left_span = left_max - left_min
        right_span = right_max - right_min
        scale_factor = float(right_span) / float(left_span)

        def tmp(value):
            return right_min + (value - left_min) * scale_factor
        return tmp

# サーボモーター制御クラス

class ServoMortors():
    def __init__(self):
        # シミュレーション上の論理的なサーボモーター角度と現実角度のOFFSET調整
        # ※サーボホーンの付け方に依存
        self.servo_theta1_offset = 90
        self.servo_theta2_offset = 140.0
        self.servo_theta3_offset = 150.0

        self.servos = []
        self.pca1 = None
        self.pca2 = None

        self.stand = False

    def deinit_servo_driver(self):
        self.pca1.deinit()
        self.pca2.deinit()

    def init_servo_driver(self):
        # I2C、PWMサーボモータードライバーの設定
        # 3個✕6関節=18個のモーターを制御するため、PCA9685搭載ボードを2枚利用
        # 注) アドレスがコンフリクトしないように、ハードウェア的に設定が必要
        i2c = busio.I2C(SCL, SDA)
        self.pca1 = PCA9685(i2c, address=0x40)
        self.pca1.frequency = 50
        self.pca2 = PCA9685(i2c, address=0x41)
        self.pca2.frequency = 50
        # サーボモーター制御クラスを初期化
        # 今回はDS3218(270度)とDS3218(180度)を利用
        # 注) パルス幅、ActuationRangeはサーボモーターの種類によって要変更
        self.servos = []
        for i in range(6):
            tmp = servo.Servo(
                self.pca1.channels[i],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=270)
            self.servos.append(tmp)
        for i in range(6):
            tmp = servo.Servo(self.pca1.channels[6 + i],
                              min_pulse=500, max_pulse=2500, actuation_range=180)
            self.servos.append(tmp)
        for i in range(6):
            tmp = servo.Servo(
                self.pca2.channels[i],
                min_pulse=500,
                max_pulse=2500,
                actuation_range=180)
            self.servos.append(tmp)

    def _get_servo(self, joint_id, group_id):
        TYPE_A = 0
        TYPE_B = 6
        TYPE_C = 12
        if joint_id == JointType.J3:
            return self.servos[TYPE_A + group_id]
        if joint_id == JointType.J2:
            return self.servos[TYPE_B + group_id]
        if joint_id == JointType.J1:
            return self.servos[TYPE_C + group_id]
        raise Exception("Not found servo on mapping")

    def control(self, angles):
        theta1, theta2, theta3 = angles
        servo_theta1 = [0 for i in range(6)]
        servo_theta2 = [0 for i in range(6)]
        servo_theta3 = [0 for i in range(6)]
        # シミュレーション上の角度と、実際にサーボドライバーに指示する角度の変換
        for i in range(6):
            if (i < 3):
                servo_theta1[i] = math.radians(
                    self.servo_theta1_offset) + theta1[i]
                servo_theta2[i] = math.radians(
                    self.servo_theta2_offset) + theta2[i]
                servo_theta3[i] = math.radians(
                    self.servo_theta3_offset) - theta3[i]
            else:
                servo_theta1[i] = math.radians(
                    self.servo_theta1_offset) - (2.0 * math.pi - theta1[i])
                servo_theta2[i] = math.radians(
                    self.servo_theta2_offset) + theta2[i]
                servo_theta3[i] = math.radians(
                    self.servo_theta3_offset) - theta3[i]

        # RaspberryPi以外（開発機）ではサーボモーター制御を行わない
        if not ENABLE_SERVO:
            return

        # サーボモータードライバーへの角度指示 (実際に動かす)
        for i in range(6):
            servo = self._get_servo(JointType.J1, i)
            t = math.degrees(servo_theta1[i])
            if t >= 0 and t <= 180:
                servo.angle = t
            else:
                print(f'(ERROR) J1[{i}]={t}')
            servo = self._get_servo(JointType.J2, i)
            t = math.degrees(servo_theta2[i])
            if t >= 0 and t <= 180:
                servo.angle = t
            else:
                print(f'(ERROR) J2[{i}]={t}')
            servo = self._get_servo(JointType.J3, i)
            t = math.degrees(servo_theta3[i])
            if t >= 0 and t <= 270:
                servo.angle = t
            else:
                print(f'(ERROR) J3[{i}]={t}')

    # 休眠姿勢
    def rest_position(self):
        if not ENABLE_SERVO:
            return
        print("rest_position")
        for i in range(6):
            servo = self._get_servo(JointType.J3, i)
            servo.angle = 150
            time.sleep(0.1)
        for i in range(6):
            servo = self._get_servo(JointType.J2, i)
            servo.angle = 140
            time.sleep(0.1)
        for i in range(6):
            servo = self._get_servo(JointType.J1, i)
            servo.angle = 90
            time.sleep(0.1)
        self.stand = False

    # 立ち上がる
    def standing_position(self):
        if not ENABLE_SERVO:
            return
        if self.stand:
            return
        print("standing_position")
        start = 140
        stop = 90
        step = -10
        angle = start
        while angle >= stop:
            for i in range(6):
                servo = self._get_servo(JointType.J2, i)
                servo.angle = angle
                time.sleep(0.1)
            angle += step
        for i in range(6):
            servo = self._get_servo(JointType.J3, i)
            servo.angle = 150 + 40
            time.sleep(0.1)
        self.stand = True


class Gamepad():
    def __init__(self):
        self.joystick = None
        self.last_received = None

    def init_gamepad(self, num=0):
        pygame.init()
        self.joystick = pygame.joystick.Joystick(num)
        self.joystick.init()

    def get_last_received(self):
        return self.last_received

    def get_latest_data(self):
        joystick = self.joystick
        if joystick is None:
            return None
        if not pygame.event.get():
            return None
        data = {
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
        self.last_received = data
        return data


class Hexapod():
    def __init__(self):

        self.init_position()

    def init_position(self):
        # Hexapodの諸元
        self.side_length = 0.1
        self.long1 = [0.04 for i in range(6)]
        self.long2 = [0.08 for i in range(6)]
        self.long3 = [0.08 for i in range(6)]
        # 各サーボモーターの角度 (シミュレーション上)
        self.theta1 = [math.radians(0) for i in range(6)]
        self.theta2 = [math.radians(0) for i in range(6)]
        self.theta3 = [math.radians(0) for i in range(6)]
        # X-Y座標面 (TOP VIEWからの座標面)
        self.leg_coords_topview = [[(0, 0) for j in range(6)]
                                   for i in range(4)]
        # X-Z'座標面 (各リンク機構をSIDE VIEWしたもの)
        self.leg_coords_sideview = [
            [(0, 0) for j in range(6)] for i in range(4)]
        # 順方向計算に使うパラメータ
        self.real_coords = [(0.15, -0.08) for i in range(6)]
        self.distance = [0.15 for i in range(6)]
        self.height = [-0.08 for i in range(6)]
        # 初期位置の決定
        for group_id in range(6):
            self.calc_backward(group_id)

    # 各関節角度のExport (サーボ制御系に渡すため)
    def export_angles(self):
        return [self.theta1, self.theta2, self.theta3]

    # 逆方向計算
    # 与えられた座標に持っていくために必要な関節角度(Theta2, Theta3)の計算
    # ※Theta1は別の場所で計算
    def calc_backward(self, group_id, apply=True):
        distance = self.distance[group_id]
        height = self.height[group_id]
        (real_x, real_y) = self.real_coords[group_id]
        real_x = distance if distance is not None else real_x
        real_y = height if height is not None else real_y

        l2 = self.long2[group_id]
        l3 = self.long3[group_id]
        real_x = real_x - self.long1[group_id]
        L = math.sqrt(real_x * real_x + real_y * real_y)
        J3, B, A, J2 = None, None, None, None
        ret = None
        try:
            J3 = math.acos((l2 * l2 + l3 * l3 - L * L) / (2 * l2 * l3))
            B = math.acos((L * L + l2 * l2 - l3 * l3) / (2 * L * l2))
            A = math.fabs(math.atan(real_y / real_x))
            # A = math.atan2(real_y, real_x)
            J2 = B - A
            theta2 = J2
            theta3 = -(math.pi - J3)
            ret = (theta2, theta3)
            if apply:
                self.theta2[group_id] = theta2
                self.theta3[group_id] = theta3
        except ValueError:
            return ret
        except ZeroDivisionError:
            return ret
        return ret

    # 順方向計算
    # 与えられた構造的パラメータと角度(Theta1, Theta2等)から関節位置を算出
    def calc_forward(self):
        r = self.side_length
        step = (math.pi * 2.0) / 6

        for i in range(6):
            l1 = self.long1[i]
            l2 = self.long2[i]
            l3 = self.long3[i]

            # TOP View
            x = r * math.cos(step * i)
            y = r * math.sin(step * i)
            self.leg_coords_topview[JointType.J1][i] = (x, y)

            Ru1 = CalcUtil.rotation((l1, 0), step * i - self.theta1[i])
            x = r * math.cos(step * i) + Ru1[0]
            y = r * math.sin(step * i) + Ru1[1]
            self.leg_coords_topview[JointType.J2][i] = (x, y)

            l2_top = CalcUtil.rotation((l2, 0), self.theta2[i])
            Ru2 = CalcUtil.rotation((l2_top[0], 0), step * i - self.theta1[i])
            x = r * math.cos(step * i) + Ru1[0] + Ru2[0]
            y = r * math.sin(step * i) + Ru1[1] + Ru2[1]
            self.leg_coords_topview[JointType.J3][i] = (x, y)

            l3_top = CalcUtil.rotation(
                (l3, 0), self.theta2[i] + self.theta3[i])
            Ru3 = CalcUtil.rotation((l3_top[0], 0), step * i - self.theta1[i])
            x = r * math.cos(step * i) + Ru1[0] + Ru2[0] + Ru3[0]
            y = r * math.sin(step * i) + Ru1[1] + Ru2[1] + Ru3[1]
            self.leg_coords_topview[JointType.J4][i] = (x, y)

            # Side View
            x = 0
            y = 0
            self.leg_coords_sideview[JointType.J1][i] = (x, y)

            x = l1
            y = 0
            self.leg_coords_sideview[JointType.J2][i] = (x, y)

            Ru1 = CalcUtil.rotation((l2, 0), self.theta2[i])
            x = self.long1[i] + Ru1[0]
            y = -Ru1[1]
            self.leg_coords_sideview[JointType.J3][i] = (x, y)

            Ru1 = CalcUtil.rotation((self.long2[i], 0), self.theta2[i])
            Ru2 = CalcUtil.rotation(
                (self.long3[i], 0), self.theta2[i] + self.theta3[i])
            x = self.long1[i] + Ru1[0] + Ru2[0]
            y = - (Ru1[1] + Ru2[1])
            self.leg_coords_sideview[JointType.J4][i] = (x, y)


class Buffer():
    def __init__(self, size):
        self.buffer = [None for i in range(size)]
        self.latest_value = None
        self.size = size

    def add(self, value):
        self.latest_value = value
        new_buffer = [None for i in range(self.size)]
        for i in range(self.size - 1):
            new_buffer[i] = self.buffer[i + 1]
        new_buffer[self.size - 1] = value
        self.buffer = new_buffer

    def get_latest(self):
        return self.latest_value

    def get_moving_average(self, num):
        s = 0.0
        n = 0
        for i in range(num):
            tmp = self.buffer[self.size - 1 - i]
            if tmp is not None:
                s += tmp
                n += 1
        if n == 0:
            return None
        return s / n


class MainWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super(MainWidget, self).__init__()
        self.cnt = 0
        self.left_clicked_pos = None
        self.right_clicked_pos = None
        self.previous_right_clicked_pos = None

        self.joy_leftstick_theta_buffer = Buffer(50)
        self.joy_leftstick_mag_buffer = Buffer(50)
        self.joy_rightstick_mag_buffer = Buffer(50)

        self.joy_lx_theta = 90.0  # 初期の進行方向
        self.time_delta = 100

        self.operation_mode = 0

        # 実機の場合はモータードライバーを初期化する
        self.servo_mortors = ServoMortors()
        if ENABLE_SERVO:
            self.servo_mortors.init_servo_driver()

        # Gamepad初期化
        self.gamepad = Gamepad()
        if USE_GAMEPAD:
            print(f"{USE_GAMEPAD} {type(USE_GAMEPAD)}")
            self.gamepad.init_gamepad()

        self.hexapod = Hexapod()
        self.scale = 1000

        # 自動描画 (100msec秒毎に描画する)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.timer_func)
        self.timer.start(self.time_delta)

    def timer_func(self):
        self.update()

    def mouseMoveEvent(self, event):
        mousex = event.x()
        mousey = event.y()

    def keyPressEvent(self, event):
        if event.key() == Qt.Key_Escape:
            self.close()

    def mousePressEvent(self, event):
        # 左クリック
        if event.button() == Qt.LeftButton:
            pos = event.pos()
            self.left_clicked_pos = pos
        # 右クリック
        if event.button() == Qt.RightButton:
            pos = event.pos()
            self.right_clicked_pos = pos
        self.repaint()

    def paintEvent(self, event):
        # QPainterの中で実行するとエラーになるのでここで実行
        if USE_GAMEPAD:
            try:
                self.gamepad.get_latest_data()
            except Exception as e:
                print(e)
            magnitude = self.joy_leftstick_mag_buffer.get_latest()
            if magnitude is not None:
                self.cnt += round(CalcUtil.make_interpolater(0,
                                  100, 0, 50)(magnitude))
        else:
            # Gamepadを使わない時は機械的にカウンターを進める
            self.cnt += 50

        qp = QPainter()
        qp.begin(self)
        # qp.setRenderHint(QPainter.Antialiasing)

        JOINT_RADIUS_PX = 10
        TOP_VIEW_PANELS_X = 300
        TOP_VIEW_PANELS_Y = 300
        SIDE_VIEW_PANELS_X = 700
        SIDE_VIDE_PANELS_Y = 100
        PANEL_SIZE_X = 200
        PANEL_SIZE_Y = 150

        qp.setPen(QColor(Qt.red))
        qp.setFont(QFont('Arial', 20))
        qp.drawText(10, 50, f'cnt={self.cnt} op={self.operation_mode}')

        """
        # 左クリックで手動で関節を動かす (SIDE_VIEW_PANEL)
        for i in range(6):
            base_x = PANEL_SIZE_X * i
            base_y = PANEL_SIZE_Y
            group_id = None
            if self.left_clicked_pos is not None:
                mousex_in_global = self.left_clicked_pos.x()
                mousey_in_global = self.left_clicked_pos.y()
                mousex_in_panels = mousex_in_global - SIDE_VIEW_PANELS_X
                mousey_in_panels = mousey_in_global - SIDE_VIDE_PANELS_Y
                group_id = int(mousex_in_panels / PANEL_SIZE_X)
            if group_id == i:
                real_x = (mousex_in_panels - base_x) / self.scale
                real_y = - (mousey_in_panels - base_y) / self.scale
                self.hexapod.real_coords[group_id] = (real_x, real_y)
                self.hexapod.calc_forward()

        # 右クリックで手動で関節を動かす (Theta1への反映)
        if self.right_clicked_pos != self.previous_right_clicked_pos:
            target_joint = JointGroupId.A
            original_mousex = self.right_clicked_pos.x()
            original_mousey = self.right_clicked_pos.y()
            real_x = (original_mousex - TOP_VIEW_PANELS_X) / self.scale
            real_y = (original_mousey - TOP_VIEW_PANELS_Y) / self.scale
            (x1, y1) = self.hexapod.leg_coords_topview[JointType.J1][target_joint]
            x_diff1 = real_x - x1
            y_diff1 = real_y - y1
            self.hexapod.theta1[target_joint] = math.atan2(-y_diff1, x_diff1) + (2.0*math.pi/6.0) * target_joint
            self.hexapod.distance[target_joint] = math.sqrt(x_diff1*x_diff1 + y_diff1*y_diff1)
            self.hexapod.calc_forward()
            #qp.setPen(QPen(Qt.green, 3, Qt.SolidLine))
            #qp.drawLine(original_mousex, original_mousey,
            #            round(x1 * self.scale + TOP_VIEW_PANELS_X), round(y1 * self.scale + TOP_VIEW_PANELS_Y))
            self.previous_right_clicked_pos = self.right_clicked_pos
        """

        #
        # 足を制御する
        #
        def relu(a):
            return a if a > 0 else 0
        # 1周期の分割数 (高ければ高いほど小刻みな運動)
        num_divide = 1000
        # 一歩の歩幅 (m)
        stroke_step_length_a = 0.05
        # 一歩の高さ
        stroke_step_length_b = 0.03
        # TOP_VIEWにおける関節と足の定位置までの距離
        distance_a = 0.1
        # SIDE_VIEWから見た時の関節と地面までの距離
        distance_b = -0.12
        # よく使い回す値
        tmp_theta1 = 2.0 * math.pi / 6.0
        tmp_theta2 = 2.0 * math.pi / num_divide

        latest_left_magnitude = self.joy_leftstick_mag_buffer.get_latest()
        latest_right_magnitude = self.joy_rightstick_mag_buffer.get_latest()
        magnitude = self.joy_leftstick_mag_buffer.get_moving_average(5)
        theta = self.joy_leftstick_theta_buffer.get_moving_average(5)
        if (latest_left_magnitude is not None) and (
                latest_left_magnitude > 10.0):
            self.joy_lx_theta = theta

        # 各関節の位置計算
        for target_joint in range(6):
            # 位相角の計算
            phase_diff1 = math.pi / 2 if (target_joint % 2 != 0) else 0
            phase_diff2 = math.pi if (target_joint % 2 != 0) else 0
            # J1からdistance_a[m]離れた位置を計算する (TOP_VIEWでみた足を着地すべき場所)
            x = distance_a * math.cos(tmp_theta1 * target_joint)
            y = distance_a * math.sin(tmp_theta1 * target_joint)
            # y -= stroke_step_length_a * math.fabs(math.sin(tmp_theta2 * self.cnt + phase_diff1))

            # スティックの角度の方向にStrokeを向ける
            tmp = stroke_step_length_a * \
                math.fabs(math.sin(tmp_theta2 * self.cnt + phase_diff1))
            t = -self.joy_lx_theta
            (tmp_x1, tmp_y1) = CalcUtil.rotation((tmp, 0), math.radians(t))
            x += tmp_x1
            y += tmp_y1

            # 角度、距離などを変更
            self.hexapod.theta1[target_joint] = math.atan2(
                -y, x) + (2.0 * math.pi / 6.0) * target_joint
            self.hexapod.distance[target_joint] = math.sqrt(x * x + y * y)
            self.hexapod.height[target_joint] = distance_b + math.fabs(
                relu(stroke_step_length_b * math.sin(tmp_theta2 * 2 * self.cnt + phase_diff2)))

            # TOP VIEWにおける可動範囲の描画
            (x1,
             y1) = self.hexapod.leg_coords_topview[JointType.J1][target_joint]
            base_x = x1 + distance_a * math.cos(tmp_theta1 * target_joint)
            base_y = y1 + distance_a * math.sin(tmp_theta1 * target_joint)

            t = -self.joy_lx_theta
            (tmp_x1, tmp_y1) = CalcUtil.rotation(
                (-stroke_step_length_a, 0), math.radians(t))
            (tmp_x2, tmp_y2) = CalcUtil.rotation(
                (+stroke_step_length_a, 0), math.radians(t))
            qp.setPen(QPen(Qt.red, 3, Qt.SolidLine))
            qp.drawLine(
                round(
                    (base_x + tmp_x1) * self.scale + TOP_VIEW_PANELS_X),
                round(
                    (base_y + tmp_y1) * self.scale + TOP_VIEW_PANELS_Y),
                round(
                    (base_x + tmp_x2) * self.scale + TOP_VIEW_PANELS_X),
                round(
                    (base_y + tmp_y2) * self.scale + TOP_VIEW_PANELS_Y))

        if USE_GAMEPAD:
            data = self.gamepad.get_last_received()
            if data is not None:
                btn_x = data['btn_x']
                btn_y = data['btn_y']
                btn_a = data['btn_a']
                btn_b = data['btn_b']
                if btn_x:
                    self.operation_mode = 0
                if btn_y:
                    self.operation_mode = 1
                if btn_a:
                    self.operation_mode = 2
                if btn_b:
                    self.operation_mode = 3

        # 各関節角度の計算 (逆方向計算)
        for i in range(6):
            self.hexapod.calc_backward(i)
        # 各関節位置の計算 (順方向計算)
        self.hexapod.calc_forward()
        # 内部角度情報のエクスポート
        angles = self.hexapod.export_angles()
        # 実際のサーボモーターの制御
        if self.operation_mode == 0:
            self.servo_mortors.control(angles)
        if self.operation_mode == 1:
            self.servo_mortors.rest_position()
        if self.operation_mode == 2:
            self.servo_mortors.standing_position()

        # TOP Viewパネルの描画
        qp.save()
        qp.translate(TOP_VIEW_PANELS_X, TOP_VIEW_PANELS_Y)
        qp.setBrush(QColor(0, 0, 255))
        qp.setPen(Qt.NoPen)
        qp.drawEllipse(-5, -5, 10, 10)
        for j in range(4):
            point_r = 10
            for i, (x, y) in enumerate(self.hexapod.leg_coords_topview[j]):
                tmp_x = round(x * self.scale - point_r / 2)
                tmp_y = round(y * self.scale - point_r / 2)
                qp.setBrush(QColor(0, 0, 255))
                qp.setPen(Qt.NoPen)
                qp.drawEllipse(tmp_x, tmp_y, point_r, point_r)
                tmp_x = round(x * self.scale + 15)
                tmp_y = round(y * self.scale + 15)
                qp.setPen(QColor(Qt.red))
                qp.setFont(QFont('Arial', 10))
                if j == 3:
                    qp.drawText(tmp_x, tmp_y, f'J{j+1} ({x:.2f},{y:.2f})')
                else:
                    qp.drawText(tmp_x, tmp_y, f'J{j+1}')
        # Show direction of L-AXIS
        if USE_GAMEPAD:
            data = self.gamepad.get_last_received()
            if data is not None:
                joy_lx = data['joy_lx']
                joy_ly = data['joy_ly']
                x = CalcUtil.make_interpolater(-1.0, +1.0, -100, 100)(joy_lx)
                y = CalcUtil.make_interpolater(-1.0, +1.0, -100, 100)(joy_ly)
                magnitude = max(math.fabs(x), math.fabs(y))
                theta = math.degrees(math.atan2(-y, x))
                self.joy_leftstick_mag_buffer.add(magnitude)
                self.joy_leftstick_theta_buffer.add(theta)

                joy_rx = data['joy_rx']
                joy_ry = data['joy_ry']
                # x = CalcUtil.make_interpolater(-1.0, +1.0, -100, 100)(joy_lx)
                magnitude = CalcUtil.make_interpolater(
                    -1.0, +1.0, +100, -100)(joy_ry)
                if magnitude < 0:
                    magnitude = 0
                self.joy_rightstick_mag_buffer.add(magnitude)

                qp.setPen(QColor(Qt.red))
                qp.drawLine(0, 0, round(x), round(y))
                qp.setPen(QColor(Qt.red))
                qp.setFont(QFont('Arial', 10))
                qp.drawText(0, 20, f'x={x:.1f} y={y:.1f}')
                qp.drawText(
                    0, 40, f'magnitude={magnitude:.1f} theta={theta:.1f}')

        # Draw Hexagon
        for i in range(6):
            (x1, y1) = self.hexapod.leg_coords_topview[JointType.J1][i]
            (x2, y2) = self.hexapod.leg_coords_topview[JointType.J1][(
                i + 1) % 6]
            x1 = round(x1 * self.scale)
            y1 = round(y1 * self.scale)
            x2 = round(x2 * self.scale)
            y2 = round(y2 * self.scale)
            qp.setBrush(QColor(0, 0, 255))
            qp.setPen(QColor(Qt.blue))
            qp.drawLine(x1, y1, x2, y2)
        # Draw guide lines on Hexagon
        for i in range(6):
            distance = 0.2
            (x1, y1) = self.hexapod.leg_coords_topview[JointType.J1][i]
            x1 = round(x1 * self.scale)
            y1 = round(y1 * self.scale)
            x2 = round(
                x1 +
                distance *
                math.cos(
                    (2.0 *
                     math.pi /
                     6.0) *
                    i) *
                self.scale)
            y2 = round(
                y1 +
                distance *
                math.sin(
                    (2.0 *
                     math.pi /
                     6.0) *
                    i) *
                self.scale)
            qp.setPen(QColor(Qt.red))
            qp.drawLine(x1, y1, x2, y2)
        # Draw link between each joints
        for j in range(3):
            for i in range(6):
                (x1, y1) = self.hexapod.leg_coords_topview[j][i]
                (x2, y2) = self.hexapod.leg_coords_topview[j + 1][i]
                x1 = round(x1 * self.scale)
                y1 = round(y1 * self.scale)
                x2 = round(x2 * self.scale)
                y2 = round(y2 * self.scale)
                qp.setPen(QColor(Qt.blue))
                qp.drawLine(x1, y1, x2, y2)
        qp.restore()

        # SIDE VIEWパネルの描画
        qp.save()
        qp.translate(SIDE_VIEW_PANELS_X, SIDE_VIDE_PANELS_Y)
        for i in range(6):
            base_x = PANEL_SIZE_X * i
            base_y = PANEL_SIZE_Y
            # Y-AXIS
            qp.setPen(QColor(Qt.blue))
            qp.drawLine(base_x, 0, base_x, 300)
            # X-AXIS
            qp.setPen(QColor(Qt.blue))
            qp.drawLine(base_x, base_y, base_x + 150, base_y)
            # Ground
            tmps = [round(0.08 * self.scale)]
            qp.setPen(QColor(Qt.red))
            for tmp in tmps:
                qp.drawLine(base_x, base_y + tmp, base_x + 150, base_y + tmp)
            # Show Joints
            for j in range(4):
                (x, y) = self.hexapod.leg_coords_sideview[j][i]
                tmp_x = round(base_x + x * self.scale - JOINT_RADIUS_PX / 2)
                tmp_y = round(base_y + y * self.scale - JOINT_RADIUS_PX / 2)
                qp.setBrush(QColor(0, 0, 255))
                qp.setPen(Qt.NoPen)
                qp.drawEllipse(tmp_x, tmp_y, JOINT_RADIUS_PX, JOINT_RADIUS_PX)
                tmp_x = round(base_x + x * self.scale + 5)
                tmp_y = round(base_y + y * self.scale + 15)
                qp.setPen(QColor(Qt.red))
                qp.setFont(QFont('Arial', 10))
                qp.drawText(tmp_x, tmp_y, f'J{j+1}')
            # Show Lines between each joints
            for j in range(3):
                (x1, y1) = self.hexapod.leg_coords_sideview[j][i]
                (x2, y2) = self.hexapod.leg_coords_sideview[j + 1][i]
                qp.setBrush(Qt.black)
                qp.setPen(QColor(Qt.blue))
                qp.drawLine(
                    round(
                        base_x +
                        x1 *
                        self.scale),
                    round(
                        base_y +
                        y1 *
                        self.scale),
                    round(
                        base_x +
                        x2 *
                        self.scale),
                    round(
                        base_y +
                        y2 *
                        self.scale))
            # Show values
            qp.setPen(QColor(Qt.red))
            qp.setFont(QFont('Arial', 10))
            base_y = 300
            for i, text in enumerate(
                [
                    f'RealX={self.hexapod.real_coords[i][0]:.3f}',
                    f'RealY={self.hexapod.real_coords[i][1]:.3f}',
                    f'Distance={self.hexapod.distance[i]:.3f}',
                    f'Height={self.hexapod.height[i]:.3f}',
                    f'Theta1={math.degrees(self.hexapod.theta1[i]):.2f}',
                    f'Theta2={math.degrees(self.hexapod.theta2[i]):.2f}',
                    f'Theta3={math.degrees(self.hexapod.theta3[i]):.2f}',
                ]
            ):
                qp.drawText(base_x + 10, base_y + i * 15 + 10, text)
        qp.restore()
        qp.end()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setGeometry(300, 300, 1920, 700)
        self.setWindowTitle('Hexapod制御用')
        self.show()
        # Widget
        self.main_widget = MainWidget(parent=self)
        self.setCentralWidget(self.main_widget)
        # Menubar
        bar = self.menuBar()
        file_menu = bar.addMenu('File')
        open_action = QtWidgets.QAction('Open', self)
        close_action = QtWidgets.QAction('Close', self)
        file_menu.addAction(open_action)
        file_menu.addAction(close_action)
        edit_menu = bar.addMenu('Edit')
        undo_action = QtWidgets.QAction('Undo', self)
        redo_action = QtWidgets.QAction('Redo', self)
        edit_menu.addAction(undo_action)
        edit_menu.addAction(redo_action)
        close_action.triggered.connect(self.close)


if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    mw = MainWindow()
    mw.show()
    sys.exit(app.exec_())
