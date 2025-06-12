#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import PortHandler, PacketHandler
from ikpy.chain import Chain
from ikpy.link import DHLink, OriginLink
import time

# === 모터 ID ===
XM_IDS = [0, 1, 2]
AX_IDS = [3]

# === 포트 및 프로토콜 ===
DEV_AX = "/dev/ttyUSB0"
DEV_XM = "/dev/ttyUSB0"

ph_ax = PortHandler(DEV_AX)
pk_ax = PacketHandler(1.0)
ph_ax.openPort(); ph_ax.setBaudRate(115200)

ph_xm = PortHandler(DEV_XM)
pk_xm = PacketHandler(2.0)
ph_xm.openPort(); ph_xm.setBaudRate(115200)

ADDR_AX_TORQUE   = 24
ADDR_AX_SPEED    = 32
ADDR_AX_GOAL_POS = 30
ADDR_XM_TORQUE   = 64
ADDR_XM_SPEED    = 112
ADDR_XM_GOAL_POS = 116

# === 토크 ON 및 속도 설정 ===
AX_SPEED = 100
XM_SPEED = 100

for dxl_id in AX_IDS:
    pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
    pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_SPEED, AX_SPEED)
for dxl_id in XM_IDS:
    pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 1)
    pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_SPEED, XM_SPEED)

# === 로봇 팔 정의 ===
d1 = 0.0961
L2 = 0.12 
L3 = 0.12
L4 = 0.086

x_offset = 0.006
y_offset = 0.005

robot_chain = Chain(name="4dof_arm", links=[
    OriginLink(),
    DHLink(alpha=np.radians(90), a=0,   d=d1, theta=0),
    DHLink(alpha=0,            a=L2,  d=0,  theta=np.radians(90)),
    DHLink(alpha=0,            a=L3,  d=0,  theta=0, bounds=(0, np.radians(90))),
])

def ik_dh_v2(x, y, z, init_angle=None):
    target_position = np.array([x, y, z])
    
    if init_angle is None:
        init = [0, 0, -np.radians(45)]
    else:
        init = init_angle
        
    full_solution = robot_chain.inverse_kinematics(
        target_position=target_position,
        orientation_mode=None,
        initial_position=[0.0] + init
    )
    joint1, joint2, joint3 = full_solution[1:4]
    joint4 = -(joint2 + np.pi/2 + joint3) - np.pi/2
    return [joint1, joint2, joint3, joint4]

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def compute_fk(joint_angles):
    T = np.eye(4)
    thetas = joint_angles
    links = [
        (thetas[0], d1, 0,  np.pi/2),
        (thetas[1]+np.pi/2, 0, L2, 0),
        (thetas[2], 0, L3, 0),
        (thetas[3], 0, L4, 0)
    ]
    for th, d, a, al in links:
        T = T @ dh_transform(th, d, a, al)
    return T[:3, 3]  # 말단 좌표만 반환

def rad_to_ax(val):
    deg = np.degrees(val)
    return int(np.clip(512 - deg*1023/300, 0, 1023))

def rad_to_xm(val):
    deg = np.degrees(val)
    return int(np.clip(2048 - deg*4096/360, 0, 4095))

# === 원 경로 생성 ===
def generate_circle_path(center, radius, num_points=200, extra_points=4):
    cx, cy, cz = center
    total_points = num_points + extra_points
    angles = np.linspace(0, 2*np.pi*(1 + extra_points/num_points), total_points)
    points = [(cx + radius * np.cos(a), cy + radius * np.sin(a), cz) for a in angles]
    return points

# === 경로 따라가기 + 실제 말단 좌표 수집 ===
def follow_path(points, delay):
    actual_points = []

    if len(points) == 0:
        return actual_points

    # 첫 점으로 이동
    first_angles = ik_dh_v2(points[0][0],points[0][1], points[0][2])
    for j, ang in enumerate(first_angles):
        dxl_id = [0, 1, 2, 3][j]
        if dxl_id in AX_IDS:
            val = rad_to_ax(ang)
            pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_GOAL_POS, val)
        else:
            val = rad_to_xm(ang)
            pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_GOAL_POS, val)

    time.sleep(1.5)  # 고정된 위치에서 1초 대기
    ee_pos = compute_fk(first_angles)
    actual_points.append(tuple(ee_pos))
    print(f"000: Target = {points[0]}, EE = {tuple(np.round(ee_pos, 4))}")

    prev_angle = [0.0, 0.0, -np.radians(45)]  # 초기 각도 설정
    
    # 나머지 점 순회
    for i, pt in enumerate(points[1:], start=1):
        try:
            angles = ik_dh_v2(*pt, init_angle=prev_angle[:3])
            prev_angle = angles
            for j, ang in enumerate(angles):
                dxl_id = [0, 1, 2, 3][j]
                if dxl_id in AX_IDS:
                    val = rad_to_ax(ang)
                    pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_GOAL_POS, val)
                else:
                    val = rad_to_xm(ang)
                    pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_GOAL_POS, val)

            ee_pos = compute_fk(angles)
            actual_points.append(tuple(ee_pos))
            print(f"{i:03}: Target = {pt}, EE = {tuple(np.round(ee_pos, 4))}")
            time.sleep(delay)
        except Exception as e:
            print(f"IK 실패 또는 모터 에러: {e}")
    return actual_points


# === 시각화 ===
def visualize_trajectory(points, show_labels=False):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = zip(*points)
    ax.plot(xs, ys, zs, 'o-', label="Actual End Effector Path")
    ax.scatter(xs[0], ys[0], zs[0], color='green', label="Start")
    ax.scatter(xs[-1], ys[-1], zs[-1], color='red', label="End")

    if show_labels:
        for i, (x, y, z) in enumerate(points):
            ax.text(x, y, z, f'{i}', fontsize=8)

    ax.set_xlabel('X (m)')
    ax.set_ylabel('Y (m)')
    ax.set_zlabel('Z (m)')
    ax.set_title('Robot Arm Actual End Effector Trajectory')
    ax.legend()
    plt.show()

# === 메인 === (x값 0.13에서 가장잘됨 여기로 최대한 맞추기)
if __name__ == "__main__":
    center_point = (x_offset+0.16, y_offset-0.04, L4)  # 중심점
    radius = 0.05
    path = generate_circle_path(center_point, radius)
    followed = follow_path(path, delay=0.05)
    visualize_trajectory(followed, show_labels=True)