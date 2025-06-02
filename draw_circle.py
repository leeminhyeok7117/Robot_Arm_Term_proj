# #!/usr/bin/env python3
# # -*- coding: utf-8 -*-

# import numpy as np
# import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from dynamixel_sdk import PortHandler, PacketHandler
# from ikpy.chain import Chain
# from ikpy.link import DHLink, OriginLink
# import inspect
# import time

# # === 모터별 ID 분류 ===
# XM_IDS = [0, 1, 2]
# AX_IDS = [3]

# # === 포트 및 프로토콜 설정 ===
# DEV_AX = "/dev/ttyUSB0"
# DEV_XM = "/dev/ttyUSB0"   # 실제 포트명에 맞게 수정

# # AX (Protocol 1.0), XM (Protocol 2.0) – 둘 다 115200 bps
# ph_ax = PortHandler(DEV_AX)
# pk_ax = PacketHandler(1.0)
# ph_ax.openPort(); ph_ax.setBaudRate(115200)

# ph_xm = PortHandler(DEV_XM)
# pk_xm = PacketHandler(2.0)
# ph_xm.openPort(); ph_xm.setBaudRate(115200)

# # === 제어용 주소 정의 ===
# # AX-12A (Prot1)
# ADDR_AX_TORQUE   = 24
# ADDR_AX_SPEED    = 32
# ADDR_AX_GOAL_POS = 30
# # XM430 (Prot2)
# ADDR_XM_TORQUE   = 64
# ADDR_XM_SPEED    = 112
# ADDR_XM_GOAL_POS = 116

# # === 모터 토크 활성화 ===
# for dxl_id in AX_IDS:
#     pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
# for dxl_id in XM_IDS:
#     pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 1)

# # === 속도 설정 & 매핑 ===
# AX_SPEED       = 200    # AX 레지스터 값 (0–1023)
# XM_SPEED       = 100

# print(f"AX_SPEED={AX_SPEED}, mapped XM_SPEED={XM_SPEED}")

# # 토크 및 속도 활성화
# for dxl_id in AX_IDS:
#     pk_ax.write1ByteTxRx(ph_ax, dxl_id, ADDR_AX_TORQUE, 1)
#     pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_SPEED, AX_SPEED)

# for dxl_id in XM_IDS:
#     pk_xm.write1ByteTxRx(ph_xm, dxl_id, ADDR_XM_TORQUE, 3)
#     pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_SPEED, XM_SPEED)

# # === 링크 길이 ===
# d1 = 0.0961
# L2 = 0.12
# L3 = 0.12
# L4 = 0.12
# HOR = 0.053
# VER = 0.052

# robot_chain = Chain(name="4dof_arm", links=[
#     OriginLink(),
#     DHLink(alpha=np.radians(90), a=0,   d=d1, theta=0),
#     DHLink(alpha=0,              a=L2,  d=0,  theta=np.radians(90)),
#     # DHLink(alpha=0 ,a=0,  d=0,  theta=0, bounds=(0, np.radians(90))),
#     DHLink(alpha=0, a=HOR, d=0, theta=0)
# ])

# def ik_dh_v2(x, y, z):

#     target_position = np.array([x, y, z])
#     init = [
#         0.0,             # OriginLink (더미)
#         0.0,             # θ1 (베이스 yaw)
#         0.0,             # θ2 (어깨 pitch)
#         -np.radians(45) # θ3 (팔꿈치 pitch 초기 추정)
#     ]

#     full_solution = robot_chain.inverse_kinematics(
#         target_position=target_position,
#         orientation_mode=None,
#         initial_position=init
#     )
#     joint1, joint2, joint3 = full_solution[1:4]

#     corrected_theta4 =  -(joint2 + np.pi/2 + joint3)

#     # 최종 관절각 리스트
#     joint_angles = [joint1, joint2, joint3, corrected_theta4]

#     return joint_angles

# def dh_transform(theta, d, a, alpha):
#     ct, st = np.cos(theta), np.sin(theta)
#     ca, sa = np.cos(alpha), np.sin(alpha)
#     return np.array([
#         [ct, -st*ca,  st*sa, a*ct],
#         [st,  ct*ca, -ct*sa, a*st],
#         [0,      sa,     ca,    d],
#         [0,       0,      0,    1]
#     ])

# def rad_to_ax(val):
#     deg = np.degrees(val)
#     return int(np.clip(512 - deg*1023/300, 0, 1023))

# def rad_to_xm(val):
#     deg = np.degrees(val)
#     return int(np.clip(2048 - deg*4096/360, 0, 4095))

# def cubic_trajectory(p0, v0, pf, vf, T, t):
#     M = np.array([
#         [0, 0, 0, 1],
#         [T**3, T**2, T, 1],
#         [0, 0, 1, 0],
#         [3*T**2, 2*T, 1, 0]
#     ])
#     b = np.array([p0, pf, v0, vf])
#     coeffs = np.linalg.solve(M, b)
#     a3, a2, a1, a0 = coeffs
#     return a3*t**3 + a2*t**2 + a1*t + a0

# # def quintic_trajectory(p0, v0, a0, pf, vf, af, T, t):
# #     M = np.array([
# #         [0, 0, 0, 0, 0, 1],
# #         [T**5, T**4, T**3, T**2, T, 1],
# #         [0, 0, 0, 0, 1, 0],
# #         [5*T**4, 4*T**3, 3*T**2, 2*T, 1, 0],
# #         [0, 0, 0, 2, 0, 0],
# #         [20*T**3, 12*T**2, 6*T, 2, 0, 0]
# #     ])
# #     b = np.array([p0, pf, v0, vf, a0, af])
# #     coeffs = np.linalg.solve(M, b)
# #     a5, a4, a3, a2, a1, a0 = coeffs
# #     return a5*t**5 + a4*t**4 + a3*t**3 + a2*t**2 + a1*t + a0

# def generate_circle_quintic(center, radius, duration, steps):
#     x_c, y_c, z_c = center
#     points = []
#     for i in range(steps):
#         t = i * (duration / steps)
#         #theta = quintic_trajectory(0, 0, 0, 2*np.pi, 0, 0, duration, t)
#         theta = cubic_trajectory(0,0,2*np.pi,0,duration,t)
#         x =z_c
#         y = x_c + radius * np.cos(theta)
#         z = y_c + radius * np.sin(theta)
#         points.append((x, y, z))
#     return points

# def send_joint_angles(joint_angles):
#     for i, ang in enumerate(joint_angles):
#         dxl_id = [0, 1, 2, 3][i]
#         if dxl_id in AX_IDS:
#             val = rad_to_ax(ang)
#             pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_GOAL_POS, val)
#         else:
#             val = rad_to_xm(ang)
#             pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_GOAL_POS, val)

# # === 메인 ===
# if __name__ == "__main__":
#     center = (0.15, 0.0, d1 + VER)
#     radius = 0.03
#     duration = 5.0
#     steps = 50

#     print("5차 다항식 기반 원형 경로 생성 중...")
#     path = generate_circle_quintic(center, radius, duration, steps)

#     fk_positions = []

#     for point in path:
#         angles = ik_dh_v2(*point)
#         send_joint_angles(angles)

#         # FK 계산
#         T1 = dh_transform(angles[0], d1, 0, np.pi/2)
#         T2 = dh_transform(angles[1], 0, L2, 0)
#         T3 = dh_transform(angles[2], 0, L3, 0)
#         T4 = dh_transform(angles[3], 0, HOR, 0)
#         final_transform = T1 @ T2 @ T3 @ T4
#         fk_positions.append(final_transform[:3, 3])

#         time.sleep(duration / steps)

#     print("원형 경로 실행 완료.")

#     # === 시각화 ===
#     fk_positions = np.array(fk_positions)
#     fig = plt.figure()
#     ax = fig.add_subplot(111, projection='3d')
#     ax.plot(fk_positions[:, 0], fk_positions[:, 1], fk_positions[:, 2],
#             label='End-effector Trajectory', marker='o')
#     ax.scatter(*center, color='red', label='Target Circle Center')
#     ax.set_xlabel('X (m)')
#     ax.set_ylabel('Y (m)')
#     ax.set_zlabel('Z (m)')
#     ax.set_title('End-Effector Path (FK)')
#     ax.legend()
#     ax.set_box_aspect([1,1,1])
#     plt.show()



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
x_offset = 0.007
d1 = 0.0961
L2 = 0.12 
L3 = 0.12
L4 = 0.12
HOR = 0.053
VER = 0.052

robot_chain = Chain(name="4dof_arm", links=[
    OriginLink(),
    DHLink(alpha=np.radians(90), a=0,   d=d1, theta=0),
    DHLink(alpha=0,            a=L2,  d=0,  theta=np.radians(90)),
    DHLink(alpha=0,            a=L3,  d=0,  theta=0, bounds=(0, np.radians(90))),
])

def ik_dh_v2(x, y, z):
    target_position = np.array([x, y, z])
    init = [0.0, 0.0, -np.radians(45)]
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
def generate_circle_path(center, radius, num_points=100):
    cx, cy, cz = center
    angles = np.linspace(0, 2*np.pi, num_points)
    points = [(cx + radius * np.cos(a), cy + radius * np.sin(a), cz) for a in angles]
    return points

# === 경로 따라가기 + 실제 말단 좌표 수집 ===
def follow_path(points, delay):
    actual_points = []
    for i, pt in enumerate(points):
        try:
            angles = ik_dh_v2(*pt)

            # 모터로 명령 전송
            for j, ang in enumerate(angles):
                dxl_id = [0, 1, 2, 3][j]
                if dxl_id in AX_IDS:
                    val = rad_to_ax(ang)
                    pk_ax.write2ByteTxRx(ph_ax, dxl_id, ADDR_AX_GOAL_POS, val)
                else:
                    val = rad_to_xm(ang)
                    pk_xm.write4ByteTxRx(ph_xm, dxl_id, ADDR_XM_GOAL_POS, val)

            # FK로 실제 말단 좌표 계산
            ee_pos = compute_fk(angles)
            actual_points.append(tuple(ee_pos))

            # 콘솔 출력
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

# === 메인 ===
if __name__ == "__main__":
    center_point = (x_offset+0.15, 0.025, d1+0.043)  # 중심점
    radius = 0.03
    path = generate_circle_path(center_point, radius)
    followed = follow_path(path, delay=0.5)
    visualize_trajectory(followed, show_labels=True)


