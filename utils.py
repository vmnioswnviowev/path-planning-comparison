# -*- coding: utf-8 -*-

# 导入必要的库
import math
import random
from constants import GAME_X, GAME_Y, GAME_WIDTH, GAME_HEIGHT, GAME_BORDER, OBSTACLE_RADIUS

# 辅助函数

def is_point_in_game_area(x, y):
    """
    检查点是否在有效的游戏区域内
    :param x: 点的 x 坐标
    :param y: 点的 y 坐标
    :return: 如果点在游戏区域内返回 True，否则返回 False
    """
    # 检查 x 和 y 坐标是否在游戏区域内（不包括边框）
    return (GAME_X < x < GAME_X + GAME_WIDTH and
            GAME_Y < y < GAME_Y + GAME_HEIGHT)

def get_random_point_in_game_area():
    """
    在游戏区域内生成一个随机点
    :return: 随机点坐标 (x, y)
    """
    # 在游戏区域内部（避开边框）生成随机坐标
    x_random = random.randint(GAME_X + GAME_BORDER, GAME_X + GAME_WIDTH - GAME_BORDER - 1)
    y_random = random.randint(GAME_Y + GAME_BORDER, GAME_Y + GAME_HEIGHT - GAME_BORDER - 1)
    return (x_random, y_random)

def get_distance(p1, p2):
    """
    计算两点之间的欧几里得距离
    :param p1: 第一个点 (x, y)
    :param p2: 第二个点 (x, y)
    :return: 两点之间的欧几里得距离
    """
    # 使用欧几里得距离公式计算两点间距离: sqrt((x2-x1)^2 + (y2-y1)^2)
    return ((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)**0.5

def get_line_points(p1, p2):
    """
    获取两点之间连线上的所有整数坐标点 (Bresenham's line algorithm 实现)
    :param p1: 起点 (x, y)
    :param p2: 终点 (x, y)
    :return: 连线上的所有整数坐标点列表
    """
    # 初始化起点和终点坐标
    x1, y1 = p1
    x2, y2 = p2
    # 初始化点列表
    points = []
    # 计算 x 和 y 方向的增量
    dx, dy = abs(x2 - x1), -abs(y2 - y1)
    # 确定 x 和 y 方向的步长（1 或 -1）
    sx, sy = 1 if x1 < x2 else -1, 1 if y1 < y2 else -1
    # 初始化误差项
    err = dx + dy
    # 循环直到到达终点
    while True:
        # 将当前点添加到列表中
        points.append((x1, y1))
        # 如果到达终点则退出循环
        if x1 == x2 and y1 == y2:
            break
        # 计算误差的两倍
        e2 = 2 * err
        # 根据误差调整 x 坐标
        if e2 >= dy:
            err += dy
            x1 += sx
        # 根据误差调整 y 坐标
        if e2 <= dx:
            err += dx
            y1 += sy
    return points

# 碰撞检测函数
def is_collision_free(p1, p2, obstacles):
    """
    检查两点之间的路径是否无碰撞（即路径上没有障碍物）
    :param p1: 起点 (x, y)
    :param p2: 终点 (x, y)
    :param obstacles: 障碍物列表，每个障碍物是一个坐标点 (x, y)
    :return: 如果路径无碰撞返回 True，否则返回 False
    """
    try:
        # 获取两点之间连线上的所有整数坐标点
        path_points = get_line_points(p1, p2)
        # 遍历路径上的每个点
        for p in path_points:
            # 确保检测点在屏幕内
            if not (0 <= p[0] < GAME_X + GAME_WIDTH and 0 <= p[1] < GAME_Y + GAME_HEIGHT):
                return False
            # 检查该点是否与任何障碍物重合或足够接近
            for obstacle in obstacles:
                if get_distance(p, obstacle) < OBSTACLE_RADIUS:
                    return False
        # 如果路径上所有点都不是障碍物，则无碰撞
        return True
    except Exception as e:
        print(f"Error in is_collision_free: {e}")
        return False

# 椭圆约束采样相关函数
def get_random_point_in_ellipse(focus1, focus2, major_axis_length):
    """
    在以focus1和focus2为焦点，major_axis_length为长轴长度的椭圆内生成随机点
    :param focus1: 第一个焦点坐标 (x, y)
    :param focus2: 第二个焦点坐标 (x, y)
    :param major_axis_length: 椭圆长轴长度
    :return: 椭圆内的随机点坐标 (x, y)
    """
    try:
        # 输入验证
        if not all(isinstance(coord, (int, float)) for coord in focus1 + focus2):
            raise ValueError("焦点坐标必须是数字")
        if major_axis_length <= 0:
            raise ValueError("长轴长度必须为正数")
        
        # 计算两个焦点之间的距离的一半
        c = get_distance(focus1, focus2) / 2
        
        # 确保长轴长度大于等于两焦点之间的距离
        if major_axis_length < 2 * c:
            major_axis_length = 2 * c
        
        # 计算椭圆的半长轴和半短轴
        a = major_axis_length / 2
        b = math.sqrt(a**2 - c**2) if a > c else 0
        
        # 处理退化椭圆（线段）
        if b == 0:
            t = random.uniform(0, 1)
            x = focus1[0] + t * (focus2[0] - focus1[0])
            y = focus1[1] + t * (focus2[1] - focus1[1])
            return (int(x), int(y))
        
        # 计算椭圆中心
        center = ((focus1[0] + focus2[0]) / 2, (focus1[1] + focus2[1]) / 2)
        
        # 计算焦点连线的角度
        dx = focus2[0] - focus1[0]
        dy = focus2[1] - focus1[1]
        angle = math.atan2(dy, dx)
        
        # 在单位圆内随机采样
        theta = random.uniform(0, 2 * math.pi)
        r = math.sqrt(random.uniform(0, 1))
        
        # 转换为椭圆内的点
        x_ellipse = a * r * math.cos(theta)
        y_ellipse = b * r * math.sin(theta)
        
        # 旋转椭圆点到正确的方向
        x_rotated = x_ellipse * math.cos(angle) - y_ellipse * math.sin(angle)
        y_rotated = x_ellipse * math.sin(angle) + y_ellipse * math.cos(angle)
        
        # 平移到椭圆中心
        x = center[0] + x_rotated
        y = center[1] + y_rotated
        
        # 确保点在游戏区域内
        x = max(GAME_X + GAME_BORDER, min(x, GAME_X + GAME_WIDTH - GAME_BORDER - 1))
        y = max(GAME_Y + GAME_BORDER, min(y, GAME_Y + GAME_HEIGHT - GAME_BORDER - 1))
        
        return (int(x), int(y))
    except Exception as e:
        print(f"Error in get_random_point_in_ellipse: {e}")
        return get_random_point_in_game_area()

def get_adaptive_random_point(start_node, end_node, path_length, use_ellipse=True):
    """
    根据是否启用椭圆约束，返回适当的随机采样点
    :param start_node: 起点坐标 (x, y)
    :param end_node: 终点坐标 (x, y)
    :param path_length: 当前路径长度
    :param use_ellipse: 是否使用椭圆约束采样
    :return: 随机采样点坐标 (x, y)
    """
    try:
        if use_ellipse and random.random() < 0.9:  # 使用默认的ELLIPSE_PROBABILITY值
            # 在椭圆内采样
            return get_random_point_in_ellipse(start_node, end_node, path_length)
        else:
            # 全局随机采样
            return get_random_point_in_game_area()
    except Exception as e:
        print(f"Error in get_adaptive_random_point: {e}")
        return get_random_point_in_game_area()

def reduce_path_points(path_points, obstacles):
    """
    删减路径点，保持路径无碰撞的情况下缩短路径
    :param path_points: 原始路径点列表（从起点到终点）
    :param obstacles: 障碍物列表
    :return: 删减后的路径点列表
    """
    if len(path_points) <= 2:
        return path_points  # 如果路径已经很短，无需删减
    
    reduced_points = [path_points[0]]  # 保留起点
    i = 0  # 当前点索引
    
    while i < len(path_points) - 1:
        j = len(path_points) - 1  # 尝试直接连接到终点
        
        while j > i + 1:
            # 检查当前点到j点是否无碰撞
            if is_collision_free(path_points[i], path_points[j], obstacles):
                # 如果可以直接连接，则跳过中间点
                reduced_points.append(path_points[j])
                i = j  # 移动到j点继续处理
                break
            j -= 1
        
        # 如果无法跳过任何点，至少前进一步
        if j == i + 1:
            reduced_points.append(path_points[i + 1])
            i += 1
    
    return reduced_points