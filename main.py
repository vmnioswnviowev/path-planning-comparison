# -*- coding: utf-8 -*-

# 导入必要的库
import pygame
import sys
import heapq
import random
import os
from datetime import datetime

# 创建截图保存目录
screenshot_dir = "algorithm_screenshots"
os.makedirs(screenshot_dir, exist_ok=True)

def save_screenshot(screen, algorithm, state):
    """
    保存屏幕截图到指定目录
    
    Args:
        screen: pygame屏幕对象
        algorithm: 使用的算法名称
        state: 当前状态
    """
    # 生成带时间戳的文件名
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
    filename = f"{screenshot_dir}/{algorithm}_{state}_{timestamp}.png"
    
    try:
        pygame.image.save(screen, filename)
        print(f"截图已保存: {filename}")
        return True
    except Exception as e:
        print(f"保存截图失败: {e}")
        return False

# 导入自定义模块
from constants import (
    SCREEN_WIDTH, SCREEN_HEIGHT, GAME_WIDTH, GAME_HEIGHT, 
    BUTTON_WIDTH, BUTTON_HEIGHT, WHITE, BLACK, RED, 
    GREEN, YELLOW, BLUE, STEP_SIZE, REWIRE_RADIUS, GOAL_RADIUS, OBSTACLE_RADIUS,
    MAX_OPTIMIZATION_ITERATIONS, MAX_OPTIMIZATION_TIME
)
from classes import GameState, Button
from utils import (
    is_point_in_game_area, get_random_point_in_game_area, get_distance, 
    is_collision_free, get_adaptive_random_point, reduce_path_points
)
from astar_algorithm import game_to_grid,  heuristic, a_star_step, reconstruct_path, calculate_path_length
from rrt_star_algorithm import run_rrt_star_step
from drawing_utils import redraw_scene, draw_ui

# 添加Excel工具导入
sys.path.append('c:\\Users\\66474\\Desktop\\python\\rrt\\COMPARSION\\comparison_10_14_time_excel')
from excel_utils import excel_logger

# 主函数
def main():
    """
    程序主入口函数
    """
    try:
        # 初始化 pygame 库
        pygame.init()

        # 初始化屏幕和字体
        screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
        pygame.display.set_caption("Path Planning Algorithms Comparison")
        font = pygame.font.SysFont('segoeuisemibold', 16)
        
        # 初始化时钟对象，用于控制帧率
        clock = pygame.time.Clock()
        
        # 路径优化计时器和计数器
        optimization_start_time = 0     # 优化开始时间
        optimization_iterations = 0     # 优化迭代次数
        elapsed_time = 0                # 已优化时间
        
        # 初始化状态和数据结构
        game_state = GameState.INIT     # 当前游戏状态
        start_node = None               # 起点坐标
        end_node = None                 # 终点坐标
        parent_map = {}                 # 存储树结构的字典（用于RRT*）
        cost_map = {}                   # 存储节点成本的字典
        obstacles = []                  # 存储障碍物坐标的列表
        current_path_length = float('inf')  # 当前路径长度
        use_ellipse_sampling = False    # 是否使用椭圆约束采样
        screenshot_taken = False        # 截图标记
        
        # A*算法相关数据结构
        open_set = []                   # 开放列表（优先队列）
        closed_set = set()              # 关闭列表（集合）
        came_from = {}                  # 记录路径的字典
        g_score = {}                    # 记录到每个节点的实际代价
        f_score = {}                    # 记录到每个节点的估计代价
        path = []                       # 找到的路径
        start_grid = None               # 起点网格坐标
        end_grid = None                 # 终点网格坐标
        selected_algorithm = None       # 选择的算法 ('astar' 或 'rrtstar')
        astar_start_time = 0            # A*算法开始时间
        astar_elapsed_time = 0          # A*算法耗时
        rrt_start_time = 0              # RRT*算法开始时间
        rrt_initial_path_time = 0       # RRT*找到初始路径的时间
        initial_path_length = float('inf')  # 初始路径长度
        last_optimization_second = -1   # 上次记录优化秒数

        # 计算按钮位置（五个按钮居中显示在界面底部，间距相同）
        num_buttons = 5  # 按钮数量
        button_gap = 15  # 按钮之间的间距
        total_buttons_width = num_buttons * BUTTON_WIDTH + (num_buttons - 1) * button_gap  # 总宽度 = 按钮宽度之和 + 间距之和
        start_x = (SCREEN_WIDTH - total_buttons_width) // 2  # 起始X坐标，确保整体居中
        button_y = SCREEN_HEIGHT - BUTTON_HEIGHT - 10  # 底部留出10像素边距
        
        # 创建UI元素
        mode_button = Button(start_x, button_y, BUTTON_WIDTH, BUTTON_HEIGHT, BLACK)
        # 创建算法选择按钮
        astar_button = Button(start_x + BUTTON_WIDTH + button_gap, button_y, BUTTON_WIDTH, BUTTON_HEIGHT, YELLOW)
        rrt_button = Button(start_x + 2 * (BUTTON_WIDTH + button_gap), button_y, BUTTON_WIDTH, BUTTON_HEIGHT, GREEN)
        rrtstar_button = Button(start_x + 3 * (BUTTON_WIDTH + button_gap), button_y, BUTTON_WIDTH, BUTTON_HEIGHT, BLUE)
        # 添加重置按钮 - 与其他按钮均匀分布
        reset_button = Button(start_x + 4 * (BUTTON_WIDTH + button_gap), button_y, BUTTON_WIDTH, BUTTON_HEIGHT, RED)

        # 主循环
        running = True
        while running:
            try:
                # 限制帧率为60FPS，确保程序不会运行过快
                clock.tick(60)
                # 事件处理循环
                for event in pygame.event.get():
                    # 处理退出事件
                    if event.type == pygame.QUIT:
                        running = False
                        break

                    # 处理鼠标点击事件
                    if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                        mouse_pos = event.pos
                        
                        # 重置按钮点击检测
                        if reset_button.is_clicked(mouse_pos):
                            # 重置所有必要的变量和状态
                            game_state = GameState.DRAW_OBSTACLES
                            start_node = None
                            end_node = None
                            parent_map = {}
                            cost_map = {}
                            obstacles = []
                            current_path_length = float('inf')
                            use_ellipse_sampling = False
                            open_set = []
                            closed_set = set()
                            came_from = {}
                            g_score = {}
                            f_score = {}
                            path = []
                            start_grid = None
                            end_grid = None
                            selected_algorithm = None
                            astar_start_time = 0
                            astar_elapsed_time = 0
                            rrt_start_time = 0
                            rrt_initial_path_time = 0
                            initial_path_length = float('inf')
                            last_optimization_second = -1
                            # 重置优化参数
                            optimization_start_time = 0
                            optimization_iterations = 0
                            elapsed_time = 0
                            # 重置优化标记
                            if hasattr(main, 'point_optimized'):
                                delattr(main, 'point_optimized')
                            # 重置截图标记
                            screenshot_taken = False
                            print("程序已重置，可以重新开始绘制障碍物")
                        
                        # 点击按钮 -> 切换状态
                        elif mode_button.is_clicked(mouse_pos):
                            # 根据当前状态切换到下一个状态
                            if game_state == GameState.DRAW_OBSTACLES:
                                game_state = GameState.SET_START
                            elif game_state == GameState.SET_START and start_node:
                                game_state = GameState.SET_END
                            elif game_state == GameState.SET_END and end_node:
                                game_state = GameState.SELECT_ALGORITHM
                        # 算法选择按钮点击检测 - 修改部分
                        # 移除对game_state的限制，只要已设置起点和终点就可以切换算法
                        elif astar_button.is_clicked(mouse_pos):
                            # 只要已设置起点和终点，就可以运行A*算法
                            if start_node and end_node:
                                selected_algorithm = 'astar'
                                # 初始化A*算法数据结构，保留障碍物、起点和终点
                                open_set = []
                                closed_set = set()
                                came_from = {}
                                g_score = {}
                                f_score = {}
                                path = []
                                # 转换游戏坐标为网格坐标
                                start_grid = game_to_grid(start_node)
                                end_grid = game_to_grid(end_node)
                                # 初始化开放列表，包含起点
                                start_h = heuristic(start_grid, end_grid)
                                # 使用随机数作为第二个排序键，避免比较节点坐标
                                heapq.heappush(open_set, (start_h, random.random(), start_grid))
                                g_score[start_grid] = 0
                                f_score[start_grid] = start_h
                                # 重置优化相关变量
                                optimization_start_time = 0
                                optimization_iterations = 0
                                elapsed_time = 0
                                use_ellipse_sampling = False
                                current_path_length = float('inf')
                                astar_start_time = pygame.time.get_ticks() / 1000.0  # 记录A*算法开始时间
                                # 重置优化标记
                            if hasattr(main, 'point_optimized'):
                                delattr(main, 'point_optimized')
                            # 重置截图标记
                            screenshot_taken = False
                            game_state = GameState.RUNNING_ASTAR
                        elif rrt_button.is_clicked(mouse_pos):
                            # 只要已设置起点和终点，就可以运行RRT算法
                            if start_node and end_node:
                                selected_algorithm = 'rrt'
                                # 重置RRT相关数据结构，但保留障碍物、起点和终点
                                parent_map = {start_node: None}  # 重新初始化父节点映射，保留起点
                                # 重置优化相关变量
                                optimization_start_time = 0
                                optimization_iterations = 0
                                elapsed_time = 0
                                use_ellipse_sampling = False
                                current_path_length = float('inf')
                                last_optimization_second = -1  # 添加这行来重置优化时间计数器
                                # 记录RRT算法开始时间
                                rrt_start_time = pygame.time.get_ticks() / 1000.0
                                # 重置优化标记
                            if hasattr(main, 'point_optimized'):
                                delattr(main, 'point_optimized')
                            # 重置RRT迭代计数器
                            if hasattr(main, 'rrt_iterations'):
                                delattr(main, 'rrt_iterations')
                            # 重置截图标记
                            screenshot_taken = False
                            game_state = GameState.RUNNING_RRT
                        elif rrtstar_button.is_clicked(mouse_pos):
                            # 只要已设置起点和终点，就可以运行RRT*算法
                            if start_node and end_node:
                                selected_algorithm = 'rrtstar'
                                # 重置RRT*相关数据结构，但保留障碍物、起点和终点
                                parent_map = {start_node: None}  # 重新初始化父节点映射，保留起点
                                cost_map = {start_node: 0}      # 重新初始化成本映射，保留起点成本
                                # 重置优化相关变量
                                optimization_start_time = 0
                                optimization_iterations = 0
                                elapsed_time = 0
                                use_ellipse_sampling = False
                                current_path_length = float('inf')
                                last_optimization_second = -1  # 添加这行来重置优化时间计数器
                                # 记录RRT*算法开始时间
                                rrt_start_time = pygame.time.get_ticks() / 1000.0
                                # 重置优化标记
                            if hasattr(main, 'point_optimized'):
                                delattr(main, 'point_optimized')
                            # 重置截图标记
                            screenshot_taken = False
                            game_state = GameState.RUNNING_RRT_STAR
                        
                        # 在游戏区域内点击
                        elif is_point_in_game_area(mouse_pos[0], mouse_pos[1]):
                            # 根据当前状态处理点击事件
                            if game_state == GameState.SET_START and not start_node:
                                # 检查点击位置是否在障碍物中
                                is_in_obstacle = False
                                for obstacle in obstacles:
                                    if get_distance(obstacle, mouse_pos) < OBSTACLE_RADIUS:
                                        is_in_obstacle = True
                                        print("请再次选择起点")
                                        break
                                
                                if not is_in_obstacle:
                                    # 设置起点 
                                    start_node = mouse_pos
                                    parent_map[start_node] = None  # 起点没有父节点
                                    cost_map[start_node] = 0      # RRT* 新增：起点成本为0
                                
                            elif game_state == GameState.SET_END and not end_node:
                                is_in_obstacle = False
                                for obstacle in obstacles:
                                    if get_distance(obstacle, mouse_pos) < OBSTACLE_RADIUS:
                                        is_in_obstacle = True
                                        print("请再次选择终点")
                                        break

                                if not is_in_obstacle:
                                    # 设置终点
                                    end_node = mouse_pos
                                    # 切换到算法选择状态
                                    game_state = GameState.SELECT_ALGORITHM
                             

                    # 处理鼠标移动事件 - 实现拖动绘制障碍物
                    if game_state == GameState.DRAW_OBSTACLES:
                    # 检查鼠标左键是否按住
                        mouse_pressed = pygame.mouse.get_pressed()
                        if mouse_pressed[0]:  # 如果左键按住
                            mouse_pos = pygame.mouse.get_pos()
                        # 如果鼠标在游戏区域内，则添加障碍物
                            if is_point_in_game_area(mouse_pos[0], mouse_pos[1]):
                            # 避免重复添加相同位置的障碍物
                                obstacle_exists = False
                                for obstacle in obstacles:
                                    if get_distance(obstacle, mouse_pos) < OBSTACLE_RADIUS:
                                        obstacle_exists = True
                                        break
                                if not obstacle_exists:
                                    obstacles.append(mouse_pos)
                
                # 根据游戏状态进行不同的处理
                if game_state == GameState.INIT:
                    # 初始化状态
                    mode_button.color = BLACK
                    status_message = "点击模式按钮开始绘制障碍物"
                    game_state = GameState.DRAW_OBSTACLES  # 自动进入绘制障碍物状态
                elif game_state == GameState.DRAW_OBSTACLES:
                    # 绘制障碍物状态
                    mode_button.color = BLACK
                    status_message = "Draw obstacles, then click the button."
                
                elif game_state == GameState.SET_START:
                    # 设置起点状态
                    mode_button.color = RED
                    status_message = "Set the START point (red), then click the button."

                elif game_state == GameState.SET_END:
                    # 设置终点状态
                    mode_button.color = GREEN
                    status_message = "Set the END point (green), then click the button."

                elif game_state == GameState.SELECT_ALGORITHM:
                    # 算法选择状态
                    mode_button.color = BLACK
                    status_message = "Select an algorithm: A* (yellow) or RRT* (blue)"

                elif game_state == GameState.RUNNING_ASTAR:
                    # 运行 A* 算法状态
                    mode_button.color = YELLOW
                    status_message = "Exploring path using A*..."
                    
                    grid_width = GAME_WIDTH // 5  # 网格宽度
                    grid_height = GAME_HEIGHT // 5  # 网格高度
                    
                    # 每帧执行多次迭代以加快速度
                    for _ in range(50):
                        found, _ = a_star_step(open_set, closed_set, came_from, g_score, f_score, 
                                              start_grid, end_grid, obstacles, grid_width, grid_height)
                        if found:
                            # 重建路径
                            path = reconstruct_path(came_from, start_grid, end_grid)
                            # 计算路径长度（使用实际距离）
                            current_path_length = calculate_path_length(path)
                            # 计算A*算法耗时
                            astar_elapsed_time = pygame.time.get_ticks() / 1000.0 - astar_start_time
                            print(f"找到路径！路径长度: {current_path_length:.2f}, 耗时: {astar_elapsed_time:.3f}秒")
                            # 记录A*算法结果到Excel
                            excel_logger.log_astar_result(current_path_length, astar_elapsed_time)
                            # 立即更新状态消息，确保截图时显示正确的文字
                            status_message = f"Path found! Path length: {current_path_length:.2f}, Time: {astar_elapsed_time:.3f}s"
                            game_state = GameState.PATH_FOUND
                            break
                        elif not open_set:
                            # 没有找到路径
                            astar_elapsed_time = pygame.time.get_ticks() / 1000.0 - astar_start_time
                            print(f"无法找到路径！耗时: {astar_elapsed_time:.3f}秒")
                            # 立即更新状态消息，显示找不到路径
                            status_message = f"No path found! Time: {astar_elapsed_time:.3f}s"
                            game_state = GameState.PATH_FOUND
                            path = []
                            break

                elif game_state == GameState.RUNNING_RRT:
                    # 运行 RRT 算法状态
                    mode_button.color = GREEN
                    status_message = "Exploring path using RRT..."
                    
                    # 检查是否需要初始化RRT迭代计数器
                    if not hasattr(main, 'rrt_iterations'):
                        setattr(main, 'rrt_iterations', 0)
                    # 增加迭代计数
                    main.rrt_iterations += 1
                    
                    # 检查是否超过最大迭代次数
                    if main.rrt_iterations > 10000:  # 设置10000次为最大迭代限制
                        print(f"RRT算法达到最大迭代次数({main.rrt_iterations})，无法找到路径！")
                        game_state = GameState.PATH_FOUND
                        current_path_length = float('inf')
                        # 重置迭代计数
                        delattr(main, 'rrt_iterations')
                        
                    # 定义RRT算法的run_rrt_step函数
                    def run_rrt_step(parent_map, target_point, step_size, obstacles, start_node, end_node):
                        """
                        执行单步 RRT 扩展算法
                        :param parent_map: 存储树结构的字典，键为节点坐标，值为其父节点坐标
                        :param target_point: 随机采样点 (x, y)
                        :param step_size: 扩展步长
                        :param obstacles: 障碍物列表
                        :param start_node: 起点坐标
                        :param end_node: 终点坐标
                        :return: (is_success, new_node) 是否成功扩展及新节点坐标
                        """
                        # 1. 寻找最近的节点
                        nearest_node = min(parent_map.keys(), key=lambda p: get_distance(p, target_point))

                        # 2. 计算朝向目标点的方向向量并归一化
                        direction = (target_point[0] - nearest_node[0], target_point[1] - nearest_node[1])
                        dist = get_distance(target_point, nearest_node)
                        if dist == 0:
                            return False, None

                        # 3. 生成新节点
                        new_node = (int(nearest_node[0] + direction[0] / dist * step_size),
                                    int(nearest_node[1] + direction[1] / dist * step_size))

                        # 检查新节点是否在游戏区域内以及是否已经在树中
                        if not is_point_in_game_area(new_node[0], new_node[1]) or new_node in parent_map:
                            return False, None

                        # 4. 检查路径是否无碰撞
                        if not is_collision_free(nearest_node, new_node, obstacles):
                            return False, None

                        # 5. 将新节点添加到树中
                        parent_map[new_node] = nearest_node
                        
                        # 扩展成功，返回新节点
                        return True, new_node
                    
                    # 多次迭代以加快速度，减少迭代次数以防止性能问题
                    for _ in range(5):
                        # 生成随机点，有10%的几率直接以终点为采样点
                        if random.random() < 0.1:
                            rand_point = end_node
                        else:
                            rand_point = get_random_point_in_game_area()
                        # 执行一步 RRT 扩展
                        success, new_node = run_rrt_step(parent_map, rand_point, STEP_SIZE, 
                                                       obstacles, start_node, end_node)
                        
                        # 检查是否到达终点区域
                        if success and get_distance(new_node, end_node) < GOAL_RADIUS:
                            # 检查从新节点到终点的路径是否无碰撞
                            if is_collision_free(new_node, end_node, obstacles):
                                # 将终点添加到树中
                                parent_map[end_node] = new_node
                                # 计算路径长度，添加安全保障防止死循环
                                current = end_node
                                path_length = 0
                                loop_count = 0
                                max_loop = len(parent_map) + 1  # 最大循环次数为树节点数+1
                                
                                while current != start_node and loop_count < max_loop:
                                    parent = parent_map.get(current)
                                    if parent:
                                        path_length += get_distance(current, parent)
                                        current = parent
                                        loop_count += 1
                                    else:
                                        path_length = float('inf')
                                        break
                                
                                # 检查是否出现循环或路径不完整
                                if loop_count >= max_loop:
                                    print("警告: 路径回溯可能出现循环，无法完成路径计算")
                                    path_length = float('inf')
                                
                                current_path_length = path_length
                                initial_path_length = current_path_length  # 记录初始路径长度
                                # 计算找到初始路径的时间
                                rrt_initial_path_time = pygame.time.get_ticks() / 1000.0 - rrt_start_time
                                print(f"找到初始路径！路径长度: {current_path_length:.2f}, 耗时: {rrt_initial_path_time:.3f}秒")
                                # 记录RRT算法结果到Excel
                                excel_logger.log_rrt_result(current_path_length, rrt_initial_path_time)
                                # 重置RRT迭代计数器
                                if hasattr(main, 'rrt_iterations'):
                                    delattr(main, 'rrt_iterations')
                                game_state = GameState.PATH_FOUND
                                break  # 退出RRT运行
                elif game_state == GameState.RUNNING_RRT_STAR:
                    # 运行 RRT* 算法状态
                    mode_button.color = BLUE
                    status_message = "Exploring path using RRT*..."
                    
                    # 多次迭代以加快速度
                    for _ in range(10):
                        # 生成随机点，有10%的几率直接以终点为采样点
                        if random.random() < 0.1:
                            rand_point = end_node
                        else:
                            rand_point = get_random_point_in_game_area()
                        # 执行一步 RRT* 扩展
                        success, new_node = run_rrt_star_step(parent_map, cost_map, rand_point, STEP_SIZE, 
                                                           REWIRE_RADIUS, obstacles, start_node, end_node)
                        
                        # 检查是否到达终点区域
                        if success and get_distance(new_node, end_node) < GOAL_RADIUS:
                            # 检查从新节点到终点的路径是否无碰撞
                            if is_collision_free(new_node, end_node, obstacles):
                                # 将终点添加到树中
                                parent_map[end_node] = new_node
                                cost_map[end_node] = cost_map[new_node] + get_distance(new_node, end_node)
                                current_path_length = cost_map[end_node]
                                initial_path_length = current_path_length  # 记录初始路径长度
                                # 计算找到初始路径的时间
                                rrt_initial_path_time = pygame.time.get_ticks() / 1000.0 - rrt_start_time
                                print(f"找到初始路径！路径长度: {current_path_length:.2f}, 耗时: {rrt_initial_path_time:.3f}秒")
                                game_state = GameState.PATH_FOUND
                                break  # 退出RRT*运行

                elif game_state == GameState.PATH_FOUND:
                    # 找到路径状态
                    if selected_algorithm == 'astar':
                        # A*算法找到路径后显示结果，并停留在此状态
                        status_message = f"Path found! Path length: {current_path_length:.2f}, Time: {astar_elapsed_time:.3f}s"
                        # 不立即切换到PATH_OPTIMIZED状态，保持显示路径
                        # game_state = GameState.PATH_OPTIMIZED
                        setattr(main, 'point_optimized', False)  # 重置路径点优化标记
                    elif selected_algorithm == 'rrt':
                        # RRT算法找到路径后显示结果，不进行优化
                        status_message = f"Path found! Path length: {current_path_length:.2f}, Time: {rrt_initial_path_time:.3f}s"
                        game_state = GameState.PATH_OPTIMIZED
                    else:
                        # RRT*算法找到路径后进行优化
                        status_message = f"Path found! Path length: {current_path_length:.2f}, Now optimizing..."
                        
                        # 初始化优化参数
                        optimization_start_time = pygame.time.get_ticks() / 1000.0  # 转换为秒
                        # 启用椭圆约束采样
                        use_ellipse_sampling = True
                        # 切换到优化状态
                        game_state = GameState.OPTIMIZING_PATH
                        
                        # 将初始路径截图标志设置为False，表示需要在下一帧截图
                        # 这样可以确保在下一帧完整绘制后再截图
                        if not isinstance(screenshot_taken, dict):
                            screenshot_taken = {'initial_path': False, 'optimized_path': False}
                        screenshot_taken['initial_path'] = False  # 标记需要截图
                
                elif game_state == GameState.OPTIMIZING_PATH:
                    # 路径优化状态 - 仅适用于RRT*算法
                    if selected_algorithm == 'rrtstar':
                        # 计算已经优化的时间
                        current_time = pygame.time.get_ticks() / 1000.0
                        elapsed_time = current_time - optimization_start_time
                        
                        # 检查是否达到优化限制
                        if optimization_iterations >= MAX_OPTIMIZATION_ITERATIONS or elapsed_time >= MAX_OPTIMIZATION_TIME:
                            # 计算最终优化百分比
                            if initial_path_length > 0:
                                improvement_percentage = ((initial_path_length - current_path_length) / initial_path_length) * 100
                                print(f"优化完成！总优化时间: {elapsed_time:.2f}秒, 初始路径长度: {initial_path_length:.2f}, 最终路径长度: {current_path_length:.2f}, 优化百分比: {improvement_percentage:.2f}%")
                                # 记录RRT*算法结果到Excel
                                excel_logger.log_rrtstar_result(current_path_length, elapsed_time, improvement_percentage)
                            status_message = f"Optimization complete! Iterations: {optimization_iterations}, Time: {elapsed_time:.2f}s, Final path length: {current_path_length:.2f}"
                            # 设置为优化完成状态，显示最终结果但不自动退出
                            game_state = GameState.PATH_OPTIMIZED
                        else:
                            # 继续优化路径
                            optimization_iterations += 1
                            
                            # 执行RRT*优化步骤，减少迭代次数以提高性能
                            for _ in range(5):  # 每帧执行5次优化
                                # 如果启用了椭圆约束采样，则使用椭圆内的随机点
                                if use_ellipse_sampling:
                                    rand_point = get_adaptive_random_point(start_node, end_node, cost_map[end_node], use_ellipse_sampling)
                                else:
                                    rand_point = get_random_point_in_game_area()
                                    
                                success, new_node = run_rrt_star_step(parent_map, cost_map, rand_point, STEP_SIZE, REWIRE_RADIUS, obstacles, start_node, end_node)
                            
                            # 检查路径是否更新
                            if end_node in cost_map and cost_map[end_node] < current_path_length:
                                old_path_length = current_path_length
                                current_path_length = cost_map[end_node]
                                improvement = old_path_length - current_path_length
                                improvement_percentage = (improvement / old_path_length) * 100 if old_path_length > 0 else 0
                                print(f"路径更新！新路径长度: {current_path_length:.2f}, 改善: {improvement:.2f}, 改善百分比: {improvement_percentage:.2f}%")
                            
                            # 按秒输出优化进度
                            current_second = int(elapsed_time)
                            if current_second > last_optimization_second and current_second <= MAX_OPTIMIZATION_TIME and initial_path_length > 0:
                                last_optimization_second = current_second
                                improvement_percentage = ((initial_path_length - current_path_length) / initial_path_length) * 100
                                improvement = initial_path_length - current_path_length
                                print(f"优化时间 {current_second}秒: 路径长度 {current_path_length:.2f}, 已改善 {improvement:.2f}, 总改善百分比 {improvement_percentage:.2f}%")
                            
                            status_message = f"Optimizing path... Iteration: {optimization_iterations}, Time: {elapsed_time:.2f}s, Path length: {current_path_length:.2f}"
                    else:
                        # 对于RRT算法，直接跳转到路径优化完成状态
                        status_message = f"Path optimization completed! Final path length: {current_path_length:.2f}"
                        game_state = GameState.PATH_OPTIMIZED
                
                elif game_state == GameState.PATH_OPTIMIZED:
                    # 路径点优化，进一步删减路径点
                    if not hasattr(main, 'point_optimized'):
                        try:
                            # 回溯原始路径
                            path_points = []
                            current = end_node
                            path_valid = True
                            
                            # 安全地回溯路径，防止出现None值
                            while current != start_node and current is not None:
                                # 确保current是有效的坐标点
                                if isinstance(current, tuple) and len(current) == 2:
                                    path_points.append(current)
                                current = parent_map.get(current)
                                
                                # 防止无限循环
                                if len(path_points) > len(parent_map) + 1:  # 路径长度不应该超过节点总数
                                    path_valid = False
                                    break
                            
                            # 如果成功回溯到起点，处理路径
                            if current == start_node and path_valid:
                                path_points.append(start_node)  # 添加起点
                                path_points.reverse()  # 反转列表，使路径从起点到终点
                                
                                # 删减路径点，减少冗余节点
                                try:
                                    optimized_path = reduce_path_points(path_points, obstacles)
                                    
                                    # 更新可视化路径
                                    if len(optimized_path) > 1:
                                        # 创建新的路径树结构
                                        new_parent_map = {}
                                        new_cost_map = {}
                                        new_cost_map[optimized_path[0]] = 0  # 起点成本为0
                                        
                                        for i in range(1, len(optimized_path)):
                                            # 确保路径点是有效的坐标点
                                            if (isinstance(optimized_path[i], tuple) and len(optimized_path[i]) == 2 and
                                                isinstance(optimized_path[i-1], tuple) and len(optimized_path[i-1]) == 2):
                                                new_parent_map[optimized_path[i]] = optimized_path[i-1]
                                                new_cost_map[optimized_path[i]] = new_cost_map[optimized_path[i-1]] + get_distance(optimized_path[i-1], optimized_path[i])
                                        
                                        # 更新主数据结构
                                        if end_node in new_parent_map and end_node in new_cost_map:
                                            parent_map = new_parent_map
                                            cost_map = new_cost_map
                                            current_path_length = cost_map[end_node]
                                            print(f"路径点优化完成！新路径点数量: {len(optimized_path)}, 优化后路径长度: {current_path_length:.2f}")
                                            # 记录路径点优化结果到Excel
                                            original_points_count = len(path_points)
                                            optimized_points_count = len(optimized_path)
                                            original_cost = cost_map[end_node] if end_node in cost_map else float('inf')
                                            excel_logger.log_point_optimization(selected_algorithm, original_points_count, optimized_points_count, original_cost, current_path_length)
                                except Exception as e:
                                    print(f"路径点优化出错: {e}")
                        except Exception as e:
                            print(f"路径回溯出错: {e}")
                        
                        # 设置优化完成标记
                        setattr(main, 'point_optimized', True)
                    
                    status_message = f"Path optimization completed! Final path length: {current_path_length:.2f}"
                
                elif game_state == GameState.QUIT:
                    # 退出状态
                    running = False

                # 重新绘制整个场景
                if selected_algorithm == 'astar':
                    # 对于A*算法，使用相应参数
                    redraw_scene(screen, obstacles, parent_map, cost_map, open_set, closed_set, path, start_node, end_node, game_state, selected_algorithm)
                else:
                    # 对于RRT*算法，使用相应参数
                    redraw_scene(screen, obstacles, parent_map, cost_map, [], set(), [], start_node, end_node, game_state, selected_algorithm)
                
                # 绘制UI和更新屏幕
                draw_ui(screen, font, mode_button, astar_button, rrt_button, rrtstar_button, status_message, game_state)
                # 绘制重置按钮
                reset_button.draw(screen)
                reset_text = font.render("Reset", True, WHITE)
                screen.blit(reset_text, (reset_button.rect.x + (reset_button.rect.width - reset_text.get_width()) // 2, 
                                         reset_button.rect.y + (reset_button.rect.height - reset_text.get_height()) // 2))
                pygame.display.update()
                
                # 截图逻辑处理
                if selected_algorithm == 'rrtstar':
                    # 为RRT*算法使用特殊的截图标记结构
                    if not isinstance(screenshot_taken, dict):
                        screenshot_taken = {'initial_path': False, 'optimized_path': False, 'path_pruned': False}
                    
                    # 情况1：当初始路径需要截图时（在OPTIMIZING_PATH状态的第一帧）
                    # 此时路径已经在游戏循环中完成绘制
                    if game_state == GameState.OPTIMIZING_PATH and not screenshot_taken['initial_path']:
                        # 强制更新显示，确保所有绘制都完成
                        pygame.display.flip()
                        # 添加一个小延迟确保显示完全更新
                        pygame.time.delay(100)
                        # 截图
                        state_name = "path_found"
                        save_screenshot(screen, selected_algorithm, state_name)
                        screenshot_taken['initial_path'] = True
                        print("已截取初始路径图片")
                    
                    # 情况2：当RRT*完成路径优化时（在PATH_OPTIMIZED状态）
                    if game_state == GameState.PATH_OPTIMIZED and not screenshot_taken['optimized_path']:
                        # 强制更新显示，确保所有绘制都完成
                        pygame.display.flip()
                        # 添加一个小延迟确保显示完全更新
                        pygame.time.delay(100)
                        # 截图
                        state_name = "path_optimized"
                        save_screenshot(screen, selected_algorithm, state_name)
                        screenshot_taken['optimized_path'] = True
                    
                    # 情况3：当RRT*完成路径剪裁后
                    if game_state == GameState.PATH_OPTIMIZED and hasattr(main, 'point_optimized') and main.point_optimized and not screenshot_taken['path_pruned']:
                        # 强制更新显示，确保所有绘制都完成
                        pygame.display.flip()
                        # 添加一个小延迟确保显示完全更新
                        pygame.time.delay(100)
                        # 截图
                        state_name = "path_pruned"
                        save_screenshot(screen, selected_algorithm, state_name)
                        screenshot_taken['path_pruned'] = True
                        print("已截取路径剪裁后图片")
                else:
                    # 对于A*和RRT算法，使用原来的截图逻辑
                    # 首先检查是否已经截取过图片
                    if not screenshot_taken:
                        # 定义截图的触发条件
                        # 条件1：当使用A*算法并且找到了路径时
                        is_astar_path_found = (game_state == GameState.PATH_FOUND and selected_algorithm == 'astar')
                        # 条件2：当任何算法完成路径优化时
                        is_path_optimized = (game_state == GameState.PATH_OPTIMIZED)
                        
                        # 检查是否满足任一截图条件
                        if is_astar_path_found or is_path_optimized:
                            # 根据当前状态确定截图名称中的状态标识
                            if game_state == GameState.PATH_FOUND:
                                state_name = "path_found"
                            else:
                                state_name = "path_optimized"
                            
                            # 强制更新显示，确保所有绘制都完成，特别是文字部分
                            pygame.display.flip()
                            # 添加一个小延迟确保显示完全更新
                            pygame.time.delay(100)
                            
                            # 调用截图保存函数，传入屏幕对象、算法名称和状态名称
                            save_screenshot(screen, selected_algorithm, state_name)
                            
                            # 将截图标记设置为True，确保每个算法运行只截图一次
                            screenshot_taken = True
            except Exception as e:
                print(f"Error in main loop: {e}")
                pygame.display.update()  # 确保用户能看到错误前的最后状态
                pygame.time.delay(2000)  # 延迟2秒让用户看到错误信息
                running = False

        # 在程序退出前保存Excel文件
        excel_logger.save_to_excel()
        pygame.quit()
        sys.exit()
        
    except Exception as e:
        print(f"Error in main function: {e}")
        pygame.quit()
        sys.exit()    

# 程序入口点
if __name__ == '__main__':
    main()