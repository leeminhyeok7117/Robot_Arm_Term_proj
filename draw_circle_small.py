#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from dynamixel_sdk import PortHandler, PacketHandler
from ikpy.chain import Chain
from ikpy.link import DHLink, OriginLink
import time

# === 모터 ID & 포트 설정 ===
XM_IDS = [0, 1, 2]
AX_IDS = [3]
DEV_AX = DEV_XM = "/dev/ttyUSB0"
ph_ax = PortHandler(DEV_AX); pk_ax = PacketHandler(1.0)
ph_ax.openPort(); ph_ax.setBaudRate(115200)
ph_xm = PortHandler(DEV_XM); pk_xm = PacketHandler(2.0)
ph_xm.openPort(); ph_xm.setBaudRate(115200)

# === 레지스터 ===
ADDR_AX_TORQUE, ADDR_AX_SPEED, ADDR_AX_GOAL_POS = 24, 32, 30
ADDR_XM_TORQUE, ADDR_XM_SPEED, ADDR_XM_GOAL_POS = 64, 112, 116

# === 토크 ON 및 속도 설정 ===
AX_SPEED = XM_SPEED = 100
for dxl_id in AX_IDS:
    pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
    pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_SPEED, AX_SPEED)
for dxl_id in XM_IDS:
    pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 1)
    pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_SPEED, XM_SPEED)

# === 로봇 팔 DH 파라미터 ===
d1, L2, L3, L4 = 0.0961, 0.12, 0.12, 0.083
x_offset, y_offset = 0.006, 0.005
robot_chain = Chain(name="4dof_arm", links=[
    OriginLink(),
    DHLink(alpha=np.radians(90), a=0,   d=d1, theta=0),
    DHLink(alpha=0,            a=L2, d=0,  theta=np.radians(90)),
    DHLink(alpha=0,            a=L3, d=0,  theta=0, bounds=(0, np.radians(90))),
])

def ik_dh_v2(x, y, z, init_angle=None):
    if init_angle is None:
        init = [0, 0, -np.radians(45)]
    else:
        init = init_angle
    sol = robot_chain.inverse_kinematics(
        target_position=np.array([x, y, z]),
        orientation_mode=None,
        initial_position=[0.0] + init
    )
    j1, j2, j3 = sol[1:4]
    j4 = -(j2 + np.pi/2 + j3) - np.pi/2
    return [j1, j2, j3, j4]

def dh_transform(theta, d, a, alpha):
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct, -st*ca,  st*sa, a*ct],
        [st,  ct*ca, -ct*sa, a*st],
        [0,      sa,     ca,    d],
        [0,       0,      0,    1]
    ])

def compute_fk(joints):
    T = np.eye(4)
    config = [
        (joints[0],         d1, 0,   np.pi/2),
        (joints[1]+np.pi/2, 0,  L2,   0),
        (joints[2],         0,  L3,   0),
        (joints[3],         0,  L4,   0),
    ]
    for θ, d, a, α in config:
        T = T @ dh_transform(θ, d, a, α)
    return T[:3, 3]

def rad_to_ax(rad):
    deg = np.degrees(rad)
    return int(np.clip(512 - deg * 1023 / 300, 0, 1023))

def rad_to_xm(rad):
    deg = np.degrees(rad)
    return int(np.clip(2048 - deg * 4096 / 360, 0, 4095))

# === 원 경로 생성 (원래 알고리즘) ===
def generate_circle_path(center, radius, num_points=300, extra_points=10):
    cx, cy, cz = center
    total = num_points + extra_points
    angles = np.linspace(0, 2*np.pi*(1 + extra_points/num_points), total)
    return [(cx + radius*np.cos(a),
             cy + radius*np.sin(a),
             cz) for a in angles]

# === 모터 제어 함수 (변경 없음) ===
def follow_path(points, delay):
    actual = []
    if not points:
        return actual

    # 첫 점 이동 (z+0.01 보정)
    x0, y0, z0 = points[0]
    start = ik_dh_v2(x0, y0, z0 + 0.01)
    for i, ang in enumerate(start):
        did = [0,1,2,3][i]
        if did in AX_IDS:
            pk_ax.write2ByteTxRx(ph_ax, did, ADDR_AX_GOAL_POS, rad_to_ax(ang))
        else:
            pk_xm.write4ByteTxRx(ph_xm, did, ADDR_XM_GOAL_POS, rad_to_xm(ang))
    time.sleep(1.5)
    actual.append(tuple(compute_fk(start)))

    prev = start
    for pt in points[1:]:
        try:
            angs = ik_dh_v2(pt[0], pt[1], pt[2], init_angle=prev[:3])
            prev = angs
            for j, ang in enumerate(angs):
                did = [0,1,2,3][j]
                if did in AX_IDS:
                    pk_ax.write2ByteTxRx(ph_ax, did, ADDR_AX_GOAL_POS, rad_to_ax(ang))
                else:
                    pk_xm.write4ByteTxRx(ph_xm, did, ADDR_XM_GOAL_POS, rad_to_xm(ang))
            actual.append(tuple(compute_fk(angs)))
            time.sleep(delay)
        except Exception as e:
            print("IK 실패 또는 모터 에러:", e)
    return actual

def visualize_trajectory(traj, show_labels=False):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    xs, ys, zs = zip(*traj)
    ax.plot(xs, ys, zs, 'o-')
    if show_labels:
        for i,(x,y,z) in enumerate(traj):
            ax.text(x, y, z, str(i), fontsize=8)
    ax.set_xlabel('X (m)'); ax.set_ylabel('Y (m)'); ax.set_zlabel('Z (m)')
    plt.show()

# === 메인: 150점씩 분할 → 시계 150점 → 상승 → 복귀 → 반시계 150점 ===
if __name__ == "__main__":
    center = (x_offset + 0.16, y_offset - 0.04, L4)
    radius = 0.02

    # 1) 전체 원 경로
    path = generate_circle_path(center, radius)

    # 2) '그릴 점 개수'를 300으로 한정
    num_draw = 300
    core     = path[:num_draw]
    half     = num_draw // 2  # 150

    # 3a) 첫 150점 (시계 방향)
    first_half = core[:half]

    # 3b) 뒤 150점 + 앞뒤로 5점씩 더 → 역순(반시계)
    start_idx   = max(0, half - 5)
    end_idx     = min(len(path), num_draw + 5)
    second_half = path[start_idx:end_idx][::-1]

    traj = []

    # A: 첫 반원
    traj += follow_path(first_half, delay=0.1)
    # B: Z축 상승
    x_c, y_c, z_c = first_half[-1]
    traj += follow_path([(x_c, y_c, z_c + 0.01)], delay=0.5)
    # C: 원점 복귀
    x0, y0, z0   = core[0]
    traj += follow_path([(x0, y0, z0)], delay=1.0)
    # D: 두번째 반원 (확장된 구간)
    traj += follow_path(second_half, delay=0.1)

    visualize_trajectory(traj, show_labels=True)
