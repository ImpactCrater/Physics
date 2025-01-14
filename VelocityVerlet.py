#! /usr/bin/python3
# -*- coding: utf8 -*-

import os, time, random, re, glob
from os.path import expanduser
from pathlib import Path
import math
import random
import numpy
import cv2


## Set up
#mkdir ~/Physics
#cd ~/Physics/
#python3.12 -m venv venv3.12
#source venv3.12/bin/activate
#pip install numpy
#pip install opencv-python
#deactivate

## Execution
#cd ~/Physics/
#source venv3.12/bin/activate
#python3 ~/Physics/VelocityVerlet.py"


## Paths

homePath = expanduser("~") # ホームのパスを表す記号


windowName = 'Physics'
cv2.namedWindow(windowName, cv2.WINDOW_AUTOSIZE | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_NORMAL)
cv2.moveWindow(windowName, 400, 100)
imageWidth = 1000 # ウィンドウの幅
imageHeight = 800 # ウィンドウの高さ
magnificationRatio = 200 # 表示倍率

"""定数を設定する"""
numberOfMassPoint = 5 # 質点の数
gravity = 9.80665 # 地球の重力加速度 [m/(s^2)]
springConstant = 16 # バネ定数
attenuationRatio = 0.001 # 減衰量
dt = 1 / 120 # 更新1回分の時間

"""質点の座標を初期化する"""
equilibriumDistanceBetweenFirstPairOfMassPoints = 0.9
coordinateOfPositionOfMassPointX = [2.5] # バネの付け根のx座標 [m]
coordinateOfPositionOfMassPointY = [0.4] # バネの付け根のy座標 [m]
for i in range(1, numberOfMassPoint):
    coordinateOfPositionOfMassPointX.append(coordinateOfPositionOfMassPointX[i - 1] + equilibriumDistanceBetweenFirstPairOfMassPoints / i)
    coordinateOfPositionOfMassPointY.append(coordinateOfPositionOfMassPointY[0])

"""質点の質量を初期化する"""
mass = [0.16] # 錘の質量
for i in range(numberOfMassPoint):
    mass.append(mass[i] / (1 + numpy.sqrt(i)))

"""質点の速度を初期化する"""
velocityOfMassPointX = []
velocityOfMassPointY = []
for i in range(numberOfMassPoint):
    velocityOfMassPointX.append(0.)
    velocityOfMassPointY.append(0.)

"""質点の加速度を初期化する"""
accelerationOfMassPointX = []
accelerationOfMassPointY = []
for i in range(numberOfMassPoint):
    accelerationOfMassPointX.append(0.)
    accelerationOfMassPointY.append(gravity)

"""バネの自然長を初期化する"""
springEquilibriumLength = []
for i in range(numberOfMassPoint - 1):
    springEquilibriumLength.append(numpy.sqrt((coordinateOfPositionOfMassPointX[i + 1] - coordinateOfPositionOfMassPointX[i]) ** 2 + (coordinateOfPositionOfMassPointY[i + 1] - coordinateOfPositionOfMassPointY[i]) ** 2))

"""バネの伸長率を初期化する"""
springElongationRatio = [] # バネの伸長率
for i in range(numberOfMassPoint - 1):
    springElongationRatio.append(1.)

"""バネの伸長を初期化する"""
springElongatedLengthX = [] # バネのx方向の伸長
springElongatedLengthY = [] # バネのy方向の伸長
for i in range(numberOfMassPoint - 1):
    springElongatedLengthX.append(0.)
    springElongatedLengthY.append(0.)

"""質点に加わる力を初期化する"""
forceOnMassPointX = []
forceOnMassPointY = []
for i in range(numberOfMassPoint):
    forceOnMassPointX.append(0.)
    forceOnMassPointY.append(0.)

# 背景色 (B, G, R) 形式で指定
backgroundColor = (32, 16, 16)
image = numpy.full((imageHeight, imageWidth, 3), backgroundColor, dtype=numpy.uint8)

def drawImage(image):
    """画像を初期化する"""
    image = numpy.full((imageHeight, imageWidth, 3), backgroundColor, dtype=numpy.uint8)

    """バネを描画する"""
    for i in range(numberOfMassPoint - 1):
        thickness = max(1, int(5 - (springElongationRatio[i] - 1) * 5))
        cv2.line(image, (int(coordinateOfPositionOfMassPointX[i] * magnificationRatio), int(coordinateOfPositionOfMassPointY[i] * magnificationRatio)), (int(coordinateOfPositionOfMassPointX[i +1] * magnificationRatio), int(coordinateOfPositionOfMassPointY[i +1] * magnificationRatio)), (32, 224, 32), thickness=thickness, lineType=cv2.LINE_AA)

    """質点を描画する"""
    cv2.circle(image, (int(coordinateOfPositionOfMassPointX[0] * magnificationRatio), int(coordinateOfPositionOfMassPointY[0] * magnificationRatio)), 6, (64, 64, 64), thickness=-1, lineType=cv2.LINE_AA) # BGR
    cv2.circle(image, (int(coordinateOfPositionOfMassPointX[0] * magnificationRatio), int(coordinateOfPositionOfMassPointY[0] * magnificationRatio)), 5, (32, 32, 224), thickness=-1, lineType=cv2.LINE_AA) # BGR
    cv2.circle(image, (int(coordinateOfPositionOfMassPointX[0] * magnificationRatio), int(coordinateOfPositionOfMassPointY[0] * magnificationRatio)), 3, (64, 64, 64), thickness=-1, lineType=cv2.LINE_AA) # BGR

    for i in range(1, numberOfMassPoint):
        cv2.circle(image, (int(coordinateOfPositionOfMassPointX[i] * magnificationRatio), int(coordinateOfPositionOfMassPointY[i] * magnificationRatio)), 9, (64, 64, 64), thickness=-1, lineType=cv2.LINE_AA) # BGR
        cv2.circle(image, (int(coordinateOfPositionOfMassPointX[i] * magnificationRatio), int(coordinateOfPositionOfMassPointY[i] * magnificationRatio)), 8, (32, 32, 224), thickness=-1, lineType=cv2.LINE_AA) # BGR

    """画像をウィンドウに表示する"""
    cv2.imshow(windowName, image)

drawImage(image)
keyPressed =cv2.waitKey(2000)

while True:

    # f = m * a <- ニュートンの運動方程式

    for i in range(1, numberOfMassPoint):
        """質点の位置座標の更新"""
        # 微小時間幅に関して等加速度運動と見做した場合
        # 移動距離 = 速度 * 時間 + 0.5 * 加速度 * 時間 * 時間
        coordinateOfPositionOfMassPointX[i] += (velocityOfMassPointX[i] * dt) + 0.5 * accelerationOfMassPointX[i] * dt ** 2
        coordinateOfPositionOfMassPointY[i] += (velocityOfMassPointY[i] * dt) + 0.5 * accelerationOfMassPointY[i] * dt ** 2

        """バネの伸長の更新"""
        springLengthX = coordinateOfPositionOfMassPointX[i] - coordinateOfPositionOfMassPointX[i - 1] # 現在のx方向の長さ
        springLengthY = coordinateOfPositionOfMassPointY[i] - coordinateOfPositionOfMassPointY[i - 1] # 現在のy方向の長さ
        springLength = numpy.sqrt(springLengthX ** 2 + springLengthY ** 2) # 三平方の定理で長さを求める
        springElongationRatio[i - 1] = springLength / springEquilibriumLength[i - 1] # バネの自然長に対する現在の長さの割合
        springElongatedLengthX[i - 1] = springLengthX - (springLengthX / springElongationRatio[i - 1]) # x方向の、現在の長さと元の長さとの差
        springElongatedLengthY[i - 1] = springLengthY - (springLengthY / springElongationRatio[i - 1]) # y方向の、現在の長さと元の長さとの差

    """ポテンシャル エナジーの更新"""
    for i in range(1, numberOfMassPoint - 1):
        forceOnMassPointX[i] = - springConstant * (springElongatedLengthX[i - 1] - springElongatedLengthX[i]) # フックの法則。f = - k * x
        forceOnMassPointY[i] = - springConstant * (springElongatedLengthY[i - 1] - springElongatedLengthY[i]) # フックの法則。f = - k * x

    forceOnMassPointX[numberOfMassPoint - 1] = - springConstant * springElongatedLengthX[numberOfMassPoint - 2] # フックの法則。f = - k * x
    forceOnMassPointY[numberOfMassPoint - 1] = - springConstant * springElongatedLengthY[numberOfMassPoint - 2] # フックの法則。f = - k * x
    
    for i in range(1, numberOfMassPoint):
        """加速度の更新"""
        previousAccelerationOfMassPointX = accelerationOfMassPointX[i]
        previousAccelerationOfMassPointY = accelerationOfMassPointY[i]
        accelerationOfMassPointX[i] = forceOnMassPointX[i] /  mass[i] # ニュートンの運動方程式より
        accelerationOfMassPointY[i] = forceOnMassPointY[i] /  mass[i] + gravity # ニュートンの運動方程式より

        """速度の更新"""
        velocityOfMassPointX[i] += 0.5 * (previousAccelerationOfMassPointX + accelerationOfMassPointX[i]) * dt - attenuationRatio * velocityOfMassPointX[i] # 加速度によりx方向の速度を更新する。速度の大きさに比例して速度を減衰させる
        velocityOfMassPointY[i] += 0.5 * (previousAccelerationOfMassPointY + accelerationOfMassPointY[i]) * dt - attenuationRatio * velocityOfMassPointY[i] # 加速度によりy方向の速度を更新する。速度の大きさに比例して速度を減衰させる

    """画像の描画"""
    drawImage(image)

    print('ay: {:.4f}, vy: {:.4f}, ey: {:.4f}'.format(accelerationOfMassPointY[numberOfMassPoint - 1], velocityOfMassPointY[numberOfMassPoint - 1], springElongationRatio[numberOfMassPoint - 2]))


    keyPressed =cv2.waitKey(int(1000/120))
    if keyPressed == ord("q"):
        break
cv2.destroyAllWindows()
