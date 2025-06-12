#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import PortHandler, PacketHandler
from ikpy.chain import Chain
from ikpy.link import DHLink, OriginLink
import inspect

# === 모터별 ID 분류 ===
XM_IDS = [0, 1]
AX_IDS = [2, 3]

# === 포트 및 프로토콜 설정 ===
DEV_AX = "/dev/ttyUSB0"
DEV_XM = "/dev/ttyUSB0"   # 실제 포트명에 맞게 수정

# AX (Protocol 1.0), XM (Protocol 2.0) – 둘 다 115200 bps
ph_ax = PortHandler(DEV_AX)
pk_ax = PacketHandler(1.0)
ph_ax.openPort(); ph_ax.setBaudRate(115200)

ph_xm = PortHandler(DEV_XM)
pk_xm = PacketHandler(2.0)
ph_xm.openPort(); ph_xm.setBaudRate(115200)

# === 제어용 주소 정의 ===
# AX-12A (Prot1)
ADDR_AX_TORQUE   = 24
ADDR_AX_SPEED    = 32
ADDR_AX_GOAL_POS = 30
# XM430 (Prot2)
ADDR_XM_TORQUE   = 64
ADDR_XM_SPEED    = 112
ADDR_XM_GOAL_POS = 116

# === 모터 토크 활성화 ===
for dxl_id in AX_IDS:
    pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
for dxl_id in XM_IDS:
    pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 1)

# === 속도 설정 & 매핑 ===
AX_SPEED       = 200    # AX 레지스터 값 (0–1023)
XM_SPEED       = 100

print(f"AX_SPEED={AX_SPEED}, mapped XM_SPEED={XM_SPEED}")

# 토크 및 속도 활성화
for dxl_id in AX_IDS:
    pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
    pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_SPEED, AX_SPEED)

for dxl_id in XM_IDS:
    pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 3)
    # XM은 4-byte 쓰되, 값은 비율 매핑된 XM_SPEED
    pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_SPEED, XM_SPEED)

# # === 링크 길이 ===
d1 = 0.0961
L2 = 0.12
L3 = 0.12
L4 = 0.12

robot_chain = Chain(name="4dof_arm", links=[
    OriginLink(),
    DHLink(alpha=np.radians(90), a=0,   d=d1, theta=0),
    DHLink(alpha=0,              a=L2,  d=0,  theta=np.radians(90)),
    DHLink(alpha=0,              a=L3,  d=0,  theta=0, bounds=(0, np.radians(90))),
    # DHLink(alpha=0,            a=L4,  d=0,  theta=0),
])

def ik_dh_v2(x, y, z):

    target_position = np.array([x, y, z])
    init = [
        0.0,             # OriginLink (더미)
        0.0,             # θ1 (베이스 yaw)
        0.0,             # θ2 (어깨 pitch)
        -np.radians(45) # θ3 (팔꿈치 pitch 초기 추정)
    ]

    full_solution = robot_chain.inverse_kinematics(
        target_position=target_position,
        orientation_mode=None,
        initial_position=init
    )
    joint1, joint2, joint3 = full_solution[1:4]

    corrected_theta4 =  -(joint2 + np.pi/2 + joint3) - np.pi/2

    # 3) 최종 관절각 리스트
    joint_angles = [joint1, joint2, joint3, corrected_theta4]

    one=dh_transform(joint_angles[0], d1, 0, np.pi/2)
    two=dh_transform(joint_angles[1]+np.pi/2, 0, L2, 0)
    thr=dh_transform(joint_angles[2], 0, L3, 0)
    four=dh_transform(joint_angles[3], 0, L4, 0)
    ot = np.dot(one,two)
    otr = np.dot(ot,thr)
    of = np.dot(otr,four)
    print(of)

    return joint_angles

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def rad_to_ax(val):
    deg = np.degrees(val)
    return int(np.clip(512 - deg*1023/300, 0, 1023))

def rad_to_xm(val):
    deg = np.degrees(val)
    return int(np.clip(2048 - deg*4096/360, 0, 4095))

# === 메인 ===
if __name__=="__main__":
    goal_pos = [0.1, 0.0, d1+L4]

    angles = ik_dh_v2(*goal_pos)
    print("IK 결과 (rad):", angles)

    for i, ang in enumerate(angles):
        dxl_id = [0,1,2,3][i]
        if dxl_id in AX_IDS:
            val = rad_to_ax(ang)
            pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_GOAL_POS, val)
        else:
            val = rad_to_xm(ang)
            pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_GOAL_POS, val)

    print("모터로 명령 전송 완료, 시각화 중...")
    print("끝.")
