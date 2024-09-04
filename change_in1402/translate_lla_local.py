import rospy
import numpy as np
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

def read_matrix_from_file(filename, line_number):
    with open(filename, 'r') as file:
        lines = file.readlines()
        selected_line = lines[line_number - 1].strip().split()
        matrix_values = [float(val) for val in selected_line]
        matrix = np.array(matrix_values).reshape(3, 3)
    return matrix

def geo2ecef(lat, lon, alt):
    # WGS84 ellipsoid constants
    a = 6378137.0        # Semi-major axis
    f = 1 / 298.257223563  # Flattening
    e2 = 2 * f - f ** 2  # Square of eccentricity

    # Convert lat, lon, alt to radians and meters
    lat_rad = np.radians(lat)
    lon_rad = np.radians(lon)

    # Compute N, the radius of curvature in the prime vertical
    N = a / np.sqrt(1 - e2 * np.sin(lat_rad) ** 2)

    # Compute ECEF coordinates
    x = (N + alt) * np.cos(lat_rad) * np.cos(lon_rad)
    y = (N + alt) * np.cos(lat_rad) * np.sin(lon_rad)
    z = (N * (1 - e2) + alt) * np.sin(lat_rad)

    return np.array([x, y, z])

def gnss_fused_callback(msg):
    global P2
    P2 = geo2ecef(msg.latitude, msg.longitude, msg.altitude)

def vision_pose_callback(msg):
    global P5,pos5
    P5 = np.array([msg.pose.position.x, msg.pose.position.y, msg.pose.position.z])
    pos5 = np.array([msg.pose.orientation.w,msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z])

def main():
    global P2, P5, P6
    rospy.init_node('compute_and_interpolate_node', anonymous=True)

    # 读取 R_enu_local 和 R_ecef_enu 矩阵
    R_enu_local = read_matrix_from_file('/home/hu/gvins_small/R_ecef_enu_local.txt', 1)
    R_ecef_enu = read_matrix_from_file('/home/hu/gvins_small/R_ecef_enu_local.txt', 2)

    # 假设 P1 是给定的LLA（从其他来源获取）
    P1_LLA = [22.305618000000003, 114.1800796, 17.302000001072884]  # 例如：某个经纬高
    P1 = geo2ecef(P1_LLA[0], P1_LLA[1], P1_LLA[2])

    # 订阅 "/mavros/gnss_anchor_lla" topic
    rospy.Subscriber('/ublox_driver/receiver_lla', NavSatFix, gnss_fused_callback)

    # 等待接收消息
    rospy.wait_for_message('/ublox_driver/receiver_lla', NavSatFix)

    # 计算P3
    P3 = np.linalg.inv(R_enu_local).dot(np.linalg.inv(R_ecef_enu).dot(P1 - P2))

    # 定义rotation_matrix3矩阵
    rotation_matrix3 = np.array([
        [0.0, -1.0,  0.0],
        [1.0,  0.0,  0.0],
        [0.0,  0.0,  1.0]
    ])

    # 计算P4
    P4 = rotation_matrix3.dot(P3)

    # 订阅 "/mavros/vision_pose/pose" topic
    rospy.Subscriber('/mavros/vision_pose/pose', PoseStamped, vision_pose_callback)

    # 等待接收消息
    rospy.wait_for_message('/mavros/vision_pose/pose', PoseStamped)

    P6 = P5 + P4

    # 输出P3, P4 和 P5
    rospy.loginfo("Computed P3: {}".format(P3))
    rospy.loginfo("Computed P4: {}".format(P4))
    rospy.loginfo("Computed P5: {}".format(P5))
    rospy.loginfo("Computed P6: {}".format(P6))

    # 生成轨迹文件
    generate_trajectory(P5, P6)

def generate_trajectory(P5, P6):
    # 设定四个目标点 (x, y, z)
    points = np.array([
        P5,               # 第一个点是P5
        [P5[0], P5[1], P5[2]+1.0],  # 中间点，可以根据需要修改
        [P6[0], P6[1], P6[2]+1.0],  # 中间点，可以根据需要修改
        P6                # 第四个点是P4
    ])

    # 插值参数
    points_per_second = 100
    speed = 0.5  # 每秒移动0.5米

    # 计算每两个点之间的距离
    distances = np.linalg.norm(points[1:] - points[:-1], axis=1)

    # 计算每两个点之间所需的时间
    total_time = distances / speed

    # 初始化时间序列
    t_values = []
    interpolated_points = []

    # 开始插值
    current_time = 0.0
    for i in range(len(points) - 1):
        start = points[i]
        end = points[i + 1]
        delta_t = 1.0 / points_per_second  # 每0.01秒插值一个点
        num_interpolated_points = int(total_time[i] * points_per_second)
        
        for j in range(num_interpolated_points):
            t = current_time + j * delta_t
            interp_point = start + (end - start) * (j / num_interpolated_points)
            t_values.append(t)
            interpolated_points.append(interp_point)
        
        current_time += total_time[i]

    # 默认的四元数表示（假设不进行旋转）
    q_w, q_x, q_y, q_z = pos5[0], pos5[1], pos5[2], pos5[3]

    # 输出文件
    output_file = "interpolated_trajectory.txt"

    with open(output_file, "w") as file:
        for i in range(len(t_values)):
            t = t_values[i]
            x, y, z = interpolated_points[i]
            file.write(f"{t:.6f} {x:.6f} {y:.6f} {z:.6f} {q_w:.6f} {q_x:.6f} {q_y:.6f} {q_z:.6f}")
            file.write(" 0.0" * 16 + "\n")

    print(f"插值文件已生成: {output_file}")

if __name__ == '__main__':
    try:
        P2 = None
        P5 = None
        main()
    except rospy.ROSInterruptException:
        pass
