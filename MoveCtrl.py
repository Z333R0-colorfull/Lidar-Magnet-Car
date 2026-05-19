import pygame
import socket
import struct
import math
import threading
import time
import numpy as np
import open3d as o3d

CAR_IP = "192.168.4.1"
CTRL_PORT = 8889
LIDAR_PORT = 8888

MAX_LINEAR = 1.0
MAX_ANGULAR = 3.50

SCREEN_SIZE = (800, 800)
JOYSTICK_RADIUS = 80
JOYSTICK_POS_LEFT = (200, 720)
JOYSTICK_POS_RIGHT = (600, 720)

RESOLUTION = 0.01          
MAP_SIZE_M = 8.0           
GRID_SIZE = int(MAP_SIZE_M / RESOLUTION)  
CELL_PIXEL = 1             

MAX_DIST = 8.0

MAX_DISTANCE = 8.0
ANGLE_MIN = 120
ANGLE_MAX = 240

ICP_THRESHOLD = 0.1
ICP_FITNESS_THRESHOLD = 0.8
MIN_POINTS_FOR_MATCH = 500
BUFFER_MAX_POINTS = 2000
LOCAL_MAP_RADIUS = 2.0
MAX_CONSECUTIVE_FAILURES = 10


class VirtualJoystick:
    def __init__(self, center, radius):
        self.center = center
        self.radius = radius
        self.pos = center
        self.dragging = False
        self.value = (0.0, 0.0)

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN:
            if self.is_point_inside(event.pos):
                self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
            self.pos = self.center
            self.value = (0.0, 0.0)
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            dx = event.pos[0] - self.center[0]
            dy = event.pos[1] - self.center[1]
            dist = math.hypot(dx, dy)
            if dist > self.radius:
                dx = dx * self.radius / dist
                dy = dy * self.radius / dist
            self.pos = (self.center[0] + dx, self.center[1] + dy)
            self.value = (dx / self.radius, -dy / self.radius)

    def is_point_inside(self, point):
        return math.hypot(point[0]-self.center[0], point[1]-self.center[1]) <= self.radius

    def draw(self, screen):
        pygame.draw.circle(screen, (100,100,100), self.center, self.radius, 2)
        pygame.draw.circle(screen, (200,0,0), self.pos, 15)


class LidarRingBuffer:
    def __init__(self, num_angles=360):
        self.num_angles = num_angles
        self.distances = [None] * num_angles

    def update(self, points):
        for dist, angle_deg in points:
            if dist <= 0 or dist > MAX_DIST:
                continue
            idx = int(round(angle_deg % 360))
            if idx == 360:
                idx = 0
            self.distances[idx] = dist

    def get_valid_points(self):
        return [(ang, d) for ang, d in enumerate(self.distances) if d is not None]

class GlobalGridMap:
    def __init__(self, grid_size, resolution, cell_pixel):
        self.grid_size = grid_size
        self.resolution = resolution
        self.cell_pixel = cell_pixel
        self.occupancy = np.zeros((grid_size, grid_size), dtype=np.uint8)
        self.lock = threading.Lock()

    def world_to_grid(self, x, y):
        half = self.grid_size * self.resolution / 2.0
        gx = int((x + half) / self.resolution)
        gy = int((y + half) / self.resolution)
        return gx, gy

    def grid_to_world(self, gx, gy):
        half = self.grid_size * self.resolution / 2.0
        x = gx * self.resolution - half + self.resolution/2
        y = gy * self.resolution - half + self.resolution/2
        return x, y

    def update_points(self, points_world):
        with self.lock:
            for p in points_world:
                gx, gy = self.world_to_grid(p[0], p[1])
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    self.occupancy[gx, gy] = 1

    def get_pointcloud(self, max_points=10000):
        with self.lock:
            occ_indices = np.argwhere(self.occupancy == 1)
        if len(occ_indices) == 0:
            return np.empty((0, 3))
        if len(occ_indices) > max_points:
            idx = np.random.choice(len(occ_indices), max_points, replace=False)
            occ_indices = occ_indices[idx]
        points = []
        for gx, gy in occ_indices:
            x, y = self.grid_to_world(gx, gy)
            points.append([x, y, 0.0])
        return np.array(points)

    
    def clear(self):
        with self.lock:
            self.occupancy.fill(0)
            print("全局栅格地图已清空")

    def draw(self, screen, center_x, center_y, cell_size):
        with self.lock:
            half_w = screen.get_width() // 2
            half_h = screen.get_height() // 2
            x_min_world = -half_w * self.resolution / cell_size
            x_max_world = half_w * self.resolution / cell_size
            y_min_world = -half_h * self.resolution / cell_size
            y_max_world = half_h * self.resolution / cell_size
            gx_min, gy_min = self.world_to_grid(x_min_world, y_min_world)
            gx_max, gy_max = self.world_to_grid(x_max_world, y_max_world)
            gx_min = max(0, gx_min)
            gy_min = max(0, gy_min)
            gx_max = min(self.grid_size, gx_max+1)
            gy_max = min(self.grid_size, gy_max+1)

            for gx in range(gx_min, gx_max):
                for gy in range(gy_min, gy_max):
                    if self.occupancy[gx, gy]:
                        wx, wy = self.grid_to_world(gx, gy)
                        sx = center_x + wx / self.resolution * cell_size
                        sy = center_y - wy / self.resolution * cell_size
                        rect = pygame.Rect(sx, sy, cell_size, cell_size)
                        pygame.draw.rect(screen, (200, 200, 200), rect)


def transform_points_local_to_world(points_local, pose):
    x, y, theta = pose
    dists = points_local[:, 0]
    angles_deg = points_local[:, 1]
    mask = (dists <= MAX_DISTANCE) & ((angles_deg < ANGLE_MIN) | (angles_deg > ANGLE_MAX))
    if not np.any(mask):
        return np.empty((0, 3))
    filtered = points_local[mask]
    angles_rad = np.deg2rad(filtered[:, 1]) + math.pi/2
    x_r = filtered[:, 0] * np.cos(angles_rad)
    y_r = filtered[:, 0] * np.sin(angles_rad)
    cos_t = math.cos(theta)
    sin_t = math.sin(theta)
    x_w = x + x_r * cos_t - y_r * sin_t
    y_w = y + x_r * sin_t + y_r * cos_t
    return np.stack([x_w, y_w, np.zeros_like(x_w)], axis=1)

def icp_registration(source_points, target_pcd, threshold=ICP_THRESHOLD):
    source_pcd = o3d.geometry.PointCloud()
    source_pcd.points = o3d.utility.Vector3dVector(source_points)
    reg_p2p = o3d.pipelines.registration.registration_icp(
        source_pcd, target_pcd, threshold, np.identity(4),
        o3d.pipelines.registration.TransformationEstimationPointToPoint(),
        o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=200)
    )
    return reg_p2p.transformation, reg_p2p.fitness


def parse_packet(data):
    if len(data) < 4:
        return None
    if data[0] != 0xA5 or data[1] != 0xA5:
        return None
    total_len = struct.unpack('<H', data[2:4])[0]
    if len(data) < total_len:
        return None
    point_count = struct.unpack('<H', data[16:18])[0]
    idx = 18
    points = []
    for _ in range(point_count):
        if idx + 8 > len(data):
            break
        dist, angle = struct.unpack('<ff', data[idx:idx+8])
        points.append((dist, angle))
        idx += 8
    return points


def control_loop(ctrl_sock, left_stick, right_stick, stop_event):
    clock = pygame.time.Clock()
    while not stop_event.is_set():
        vx = left_stick.value[1] * MAX_LINEAR*0.7
        vy = -left_stick.value[0] * MAX_LINEAR*0.7
        omega = -right_stick.value[0] * MAX_ANGULAR
        data = struct.pack('<fff', vx, vy, omega)
        try:
            ctrl_sock.sendall(data)
        except:
            pass
        clock.tick(30)

def lidar_receive_thread(lidar_sock, lidar_ring, slam_processor, stop_event):
    buffer = b''
    while not stop_event.is_set():
        try:
            data = lidar_sock.recv(4096)
            if not data:
                break
            buffer += data
        except socket.timeout:
            continue
        except:
            break

        while len(buffer) >= 4:
            if buffer[0] != 0xA5 or buffer[1] != 0xA5:
                buffer = buffer[1:]
                continue
            total_len = struct.unpack('<H', buffer[2:4])[0]
            if total_len < 18:
                buffer = buffer[1:]
                continue
            if len(buffer) >= total_len:
                packet = buffer[:total_len]
                buffer = buffer[total_len:]
                points = parse_packet(packet)
                if points:
                    lidar_ring.update(points)
                    slam_processor.process_frame(points)
            else:
                break


class SlamProcessor:
    def __init__(self, global_map):
        self.global_map = global_map
        self.robot_pose = [0.0, 0.0, 0.0]
        self.buffer_points = np.empty((0, 2))
        self.lock = threading.Lock()
        self.need_update_display = False
        self.consecutive_failures = 0

    def process_frame(self, points_raw):
        with self.lock:
            frame_points = np.array(points_raw)
            if len(frame_points) == 0:
                return
            self.buffer_points = np.vstack((self.buffer_points, frame_points))
            if self.buffer_points.shape[0] > BUFFER_MAX_POINTS:
                self.buffer_points = self.buffer_points[-BUFFER_MAX_POINTS:]

            if self.buffer_points.shape[0] >= MIN_POINTS_FOR_MATCH:
                self._perform_icp_and_update()

    def _perform_icp_and_update(self):
        source_world = transform_points_local_to_world(self.buffer_points, self.robot_pose)
        if source_world.shape[0] < 100:
            print("ICP 源点云点数过少，跳过")
            return

        target_pcd = o3d.geometry.PointCloud()
        center_x, center_y, _ = self.robot_pose
        global_pts = self.global_map.get_pointcloud(max_points=20000)
        if global_pts.shape[0] < 100:
            self.global_map.update_points(source_world)
            self.buffer_points = np.empty((0, 2))
            self.consecutive_failures = 0
            print("地图为空，直接添加点云，跳过ICP")
            return

        mask = (np.abs(global_pts[:,0] - center_x) < LOCAL_MAP_RADIUS) & \
               (np.abs(global_pts[:,1] - center_y) < LOCAL_MAP_RADIUS)
        local_target_pts = global_pts[mask]
        if local_target_pts.shape[0] < 100:
            local_target_pts = global_pts
        target_pcd.points = o3d.utility.Vector3dVector(local_target_pts)

        print(f"开始ICP：源点云 {source_world.shape[0]} 点，目标点云 {local_target_pts.shape[0]} 点")
        trans, fitness = icp_registration(source_world, target_pcd)

        if fitness > ICP_FITNESS_THRESHOLD:
            print(f"ICP 成功，适应度 {fitness:.3f}")
            self.consecutive_failures = 0

            cx, cy, ctheta = self.robot_pose
            T_curr = np.array([
                [math.cos(ctheta), -math.sin(ctheta), 0, cx],
                [math.sin(ctheta),  math.cos(ctheta), 0, cy],
                [0, 0, 1, 0],
                [0, 0, 0, 1]
            ])
            T_new = trans @ T_curr
            self.robot_pose[0] = T_new[0, 3]
            self.robot_pose[1] = T_new[1, 3]
            self.robot_pose[2] = math.atan2(T_new[1, 0], T_new[0, 0])
            print(f"新位姿: x={self.robot_pose[0]:.2f}, y={self.robot_pose[1]:.2f}, theta={math.degrees(self.robot_pose[2]):.1f}°")

            corrected_world = transform_points_local_to_world(self.buffer_points, self.robot_pose)
            self.global_map.update_points(corrected_world)
            self.buffer_points = np.empty((0, 2))
            self.need_update_display = True
        else:
            print(f"ICP 失败，适应度 {fitness:.3f}")
            self.buffer_points = np.empty((0, 2))
            self.consecutive_failures += 1
            print(f"连续失败次数: {self.consecutive_failures}")
            if self.consecutive_failures >= MAX_CONSECUTIVE_FAILURES:
                print(f"连续失败 {MAX_CONSECUTIVE_FAILURES} 次，清空全局地图并重置位姿")
                self.global_map.clear()
                self.robot_pose = [0.0, 0.0, 0.0]
                self.consecutive_failures = 0

    def get_pose(self):
        with self.lock:
            return self.robot_pose.copy()


def main():
    pygame.init()
    screen = pygame.display.set_mode(SCREEN_SIZE)
    pygame.display.set_caption("RC Car - 360° Lidar SLAM (ICP)")
    font = pygame.font.Font(None, 24)

    left_stick = VirtualJoystick(JOYSTICK_POS_LEFT, JOYSTICK_RADIUS)
    right_stick = VirtualJoystick(JOYSTICK_POS_RIGHT, JOYSTICK_RADIUS)

    print("请确保电脑已连接到 ESP32 AP (SSID: RC_CAR_AP, 密码: 12345678)")
    

    ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    ctrl_sock.connect((CAR_IP, CTRL_PORT))
    print("控制连接已建立")

    lidar_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    lidar_sock.settimeout(0.5)
    lidar_sock.connect((CAR_IP, LIDAR_PORT))
    print("雷达连接已建立")

    global_map = GlobalGridMap(GRID_SIZE, RESOLUTION, CELL_PIXEL)
    slam = SlamProcessor(global_map)

    lidar_ring = LidarRingBuffer()

    stop_event = threading.Event()
    threading.Thread(target=control_loop, args=(ctrl_sock, left_stick, right_stick, stop_event), daemon=True).start()
    threading.Thread(target=lidar_receive_thread, args=(lidar_sock, lidar_ring, slam, stop_event), daemon=True).start()

    clock = pygame.time.Clock()
    running = True

    center_x = SCREEN_SIZE[0] // 2
    center_y = SCREEN_SIZE[1] // 2

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            left_stick.handle_event(event)
            right_stick.handle_event(event)

        screen.fill((0, 0, 0))

        global_map.draw(screen, center_x, center_y, CELL_PIXEL)

        pose = slam.get_pose()
        for ang, dist in lidar_ring.get_valid_points():
            rad = math.radians(ang)
            lx = dist * math.sin(-rad)
            ly = dist * math.cos(rad)
            wx = pose[0] + lx * math.cos(pose[2]) - ly * math.sin(pose[2])
            wy = pose[1] + lx * math.sin(pose[2]) + ly * math.cos(pose[2])
            sx = center_x + wx / RESOLUTION * CELL_PIXEL
            sy = center_y - wy / RESOLUTION * CELL_PIXEL
            if 0 <= sx < SCREEN_SIZE[0] and 0 <= sy < SCREEN_SIZE[1]:
                pygame.draw.circle(screen, (255, 0, 0), (int(sx), int(sy)), 1)


        robot_sx = center_x + pose[0] / RESOLUTION * CELL_PIXEL
        robot_sy = center_y - pose[1] / RESOLUTION * CELL_PIXEL
        arrow_len = 20
        end_x = robot_sx + arrow_len * math.cos(pose[2])
        end_y = robot_sy - arrow_len * math.sin(pose[2])
        pygame.draw.circle(screen, (0, 255, 0), (int(robot_sx), int(robot_sy)), 6)
        pygame.draw.line(screen, (0, 255, 0), (robot_sx, robot_sy), (end_x, end_y), 3)

        left_stick.draw(screen)
        right_stick.draw(screen)

        info_text = font.render(f"Pose: ({pose[0]:.2f}, {pose[1]:.2f}, {math.degrees(pose[2]):.1f}°)", True, (0, 255, 0))
        screen.blit(info_text, (20, 20))
        vx_text = font.render(f"Vx: {left_stick.value[1]*MAX_LINEAR:.2f}", True, (0,0,128))
        vy_text = font.render(f"Vy: {left_stick.value[0]*MAX_LINEAR:.2f}", True, (0,0,128))
        omega_text = font.render(f"Omega: {right_stick.value[0]*MAX_ANGULAR:.2f}", True, (0,0,128))
        screen.blit(vx_text, (20, 50))
        screen.blit(vy_text, (20, 80))
        screen.blit(omega_text, (20, 110))

        pygame.display.flip()
        clock.tick(60)

    stop_event.set()
    ctrl_sock.close()
    lidar_sock.close()
    pygame.quit()

if __name__ == "__main__":
    main()