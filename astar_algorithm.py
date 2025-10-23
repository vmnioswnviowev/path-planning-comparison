# -*- coding: utf-8 -*-

# 导入必要的库
import heapq
import math
import random
from constants import GAME_X, GAME_Y, GAME_WIDTH, GAME_HEIGHT
from utils import get_distance, is_collision_free

# 游戏坐标转换为网格坐标
def game_to_grid(pos):
    """
    将游戏区域坐标转换为网格坐标
    :param pos: 游戏区域坐标 (x, y)
    :return: 网格坐标 (x, y)
    """
    grid_size = 5  # 统一使用5x5像素的网格大小
    grid_x = (pos[0] - GAME_X) // grid_size
    grid_y = (pos[1] - GAME_Y) // grid_size
    return (int(grid_x), int(grid_y))

# 将网格坐标转换为游戏区域坐标
def grid_to_game(grid_pos):
    """
    将网格坐标转换为游戏区域坐标
    :param grid_pos: 网格坐标 (x, y)
    :return: 游戏区域坐标 (x, y)
    """
    grid_size = 5  # 统一使用5x5像素的网格大小
    game_x = grid_pos[0] * grid_size + GAME_X + grid_size // 2
    game_y = grid_pos[1] * grid_size + GAME_Y + grid_size // 2
    return (game_x, game_y)

# 检查点是否在障碍物内
def is_in_obstacle(grid_pos, obstacles):
    """
    检查网格点是否在障碍物内
    :param grid_pos: 网格坐标 (x, y)
    :param obstacles: 障碍物列表，每个障碍物是一个坐标点 (x, y)
    :return: 如果在障碍物内返回 True，否则返回 False
    """
    game_pos = grid_to_game(grid_pos)
    OBSTACLE_RADIUS = 10  # 与原始代码保持一致的障碍物半径
    for obstacle in obstacles:
        if get_distance(game_pos, obstacle) < OBSTACLE_RADIUS:
            return True
    return False

# A* 算法的启发函数（使用曼哈顿距离或欧几里得距离）
def heuristic(a, b, use_manhattan=False):
    """
    A* 算法的启发函数
    :param a: 点 a 的坐标 (x, y)
    :param b: 点 b 的坐标 (x, y)
    :param use_manhattan: 是否使用曼哈顿距离（否则使用欧几里得距离）
    :return: 估计距离值
    """
    if use_manhattan:
        # 曼哈顿距离
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    else:
        # 欧几里得距离（实际距离的估计）
        return ((a[0] - b[0])**2 + (a[1] - b[1])**2)**0.5

# 获取所有相邻节点
def get_neighbors(node, grid_width, grid_height):
    """
    获取所有相邻节点（包括对角线）
    :param node: 当前节点坐标 (x, y)
    :param grid_width: 网格宽度
    :param grid_height: 网格高度
    :return: 相邻节点列表
    """
    neighbors = []
    # 八个方向的移动（包括对角线）
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0),  # 上下左右
                  (1, 1), (1, -1), (-1, -1), (-1, 1)]  # 对角线
    
    for dx, dy in directions:
        new_x = node[0] + dx
        new_y = node[1] + dy
        # 检查新坐标是否在网格范围内
        if 0 <= new_x < grid_width and 0 <= new_y < grid_height:
            neighbors.append((new_x, new_y))
    
    return neighbors

# A* 算法主函数
def a_star_step(open_set, closed_set, came_from, g_score, f_score, start_grid, end_grid, obstacles, grid_width, grid_height):
    """
    执行单步 A* 算法
    :param open_set: 开放列表（优先队列）
    :param closed_set: 关闭列表
    :param came_from: 记录路径的字典
    :param g_score: 记录从起点到各节点的实际成本
    :param f_score: 记录从起点经过各节点到终点的估计总成本
    :param start_grid: 起点网格坐标
    :param end_grid: 终点网格坐标
    :param obstacles: 障碍物列表
    :param grid_width: 网格宽度
    :param grid_height: 网格高度
    :return: (is_path_found, current) 是否找到路径及当前处理的节点
    """
    if not open_set:
        return False, None  # 开放列表为空，无解
    
    # 获取 f 得分最低的节点
    _, _, current = heapq.heappop(open_set)
    
    # 如果到达终点，返回成功
    if current == end_grid:
        return True, current
    
    # 将当前节点加入关闭列表
    closed_set.add(current)
    
    # 检查所有相邻节点
    for neighbor in get_neighbors(current, grid_width, grid_height):
        # 如果相邻节点在关闭列表中，跳过
        if neighbor in closed_set:
            continue
        
        # 检查相邻节点是否在障碍物内，跳过
        if is_in_obstacle(neighbor, obstacles):
            continue
        
        # 计算从起点经过当前节点到达相邻节点的成本
        # 对角线移动成本为√2，直线移动成本为1
        diagonal = abs(neighbor[0] - current[0]) == 1 and abs(neighbor[1] - current[1]) == 1
        move_cost = math.sqrt(2) if diagonal else 1
        tentative_g_score = g_score[current] + move_cost
        
        # 如果这是一个新节点或者找到了更好的路径
        if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
            # 记录路径
            came_from[neighbor] = current
            # 更新 g 得分和 f 得分
            g_score[neighbor] = tentative_g_score
            f_score[neighbor] = g_score[neighbor] + heuristic(neighbor, end_grid)
            # 将相邻节点加入开放列表（如果尚未加入）
            heapq.heappush(open_set, (f_score[neighbor], random.random(), neighbor))
    
    return False, current

# 从终点回溯路径
def reconstruct_path(came_from, start_grid, end_grid):
    """
    从终点回溯到起点，重建路径
    :param came_from: 记录路径的字典
    :param start_grid: 起点网格坐标
    :param end_grid: 终点网格坐标
    :return: 路径节点列表
    """
    path = []
    current = end_grid
    while current != start_grid:
        path.append(current)
        if current not in came_from:
            return None  # 路径不存在
        current = came_from[current]
    path.append(start_grid)  # 添加起点
    path.reverse()  # 反转路径，使其从起点到终点
    return path

# 计算路径长度（转换为游戏坐标的实际距离）
def calculate_path_length(path):
    """
    计算路径的实际长度（转换为游戏坐标）
    :param path: 路径节点列表（网格坐标）
    :return: 路径实际长度
    """
    if not path or len(path) <= 1:
        return 0
        
    length = 0
    for i in range(len(path) - 1):
        # 转换为游戏坐标计算实际距离
        p1 = grid_to_game(path[i])
        p2 = grid_to_game(path[i + 1])
        length += get_distance(p1, p2)
        
    return length
    return path