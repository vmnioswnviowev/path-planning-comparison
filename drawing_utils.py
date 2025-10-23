# -*- coding: utf-8 -*-

# 导入必要的库
import pygame
import math
from constants import (
    WHITE, BLACK, RED, GREEN, BLUE, YELLOW, PATH_COLOR, 
    OPEN_SET_COLOR, CLOSED_SET_COLOR, GAME_X, GAME_Y, GAME_WIDTH, 
    GAME_HEIGHT, GAME_BORDER, NODE_RADIUS, GOAL_RADIUS,
    OBSTACLE_RADIUS, BUTTON_WIDTH, BUTTON_HEIGHT, SCREEN_WIDTH, SCREEN_HEIGHT
)
from classes import GameState
from astar_algorithm import grid_to_game

# 重绘整个场景
def redraw_scene(screen, obstacles, parent_map, cost_map, open_set, closed_set, path, start_node, end_node, game_state, algorithm_type=None):
    """
    重新绘制整个场景
    :param screen: pygame 屏幕对象
    :param obstacles: 障碍物列表
    :param parent_map: 树结构字典 (用于RRT*)
    :param cost_map: 节点成本字典 (用于RRT*)
    :param open_set: 开放列表 (用于A*)
    :param closed_set: 关闭列表 (用于A*)
    :param path: 找到的路径 (用于A*)
    :param start_node: 起点坐标
    :param end_node: 终点坐标
    :param game_state: 当前游戏状态
    :param algorithm_type: 使用的算法类型 ('astar' 或 'rrtstar')
    """
    try:
        # 填充背景色为白色
        screen.fill(WHITE)
        
        # 绘制游戏区域边框
        pygame.draw.rect(screen, BLACK, (GAME_X, GAME_Y, GAME_WIDTH, GAME_HEIGHT), GAME_BORDER)
        
        # 绘制所有障碍物
        for obstacle in obstacles:
            pygame.draw.circle(screen, BLACK, obstacle, OBSTACLE_RADIUS)
        
        # 绘制起点和终点（如果已设置）
        if start_node:
            pygame.draw.circle(screen, RED, start_node, GOAL_RADIUS)
        if end_node:
            pygame.draw.circle(screen, GREEN, end_node, GOAL_RADIUS)
        
        # 绘制 A* 搜索状态
        if game_state in [GameState.RUNNING_ASTAR, GameState.PATH_FOUND, GameState.QUIT] or algorithm_type == 'astar':
            # 绘制关闭列表中的节点
            for node in closed_set:
                game_pos = grid_to_game(node)
                pygame.draw.circle(screen, CLOSED_SET_COLOR, game_pos, NODE_RADIUS)
            
            # 绘制开放列表中的节点
            open_nodes = set()
            for _, _, node in open_set:
                open_nodes.add(node)
            for node in open_nodes:
                if node not in closed_set:  # 确保不重复绘制
                    game_pos = grid_to_game(node)
                    pygame.draw.circle(screen, OPEN_SET_COLOR, game_pos, NODE_RADIUS)
            
            # 如果找到路径，绘制路径
            if path and game_state in [GameState.PATH_FOUND, GameState.QUIT]:
                for i in range(len(path) - 1):
                    p1 = grid_to_game(path[i])
                    p2 = grid_to_game(path[i + 1])
                    pygame.draw.line(screen, PATH_COLOR, p1, p2, 3)
        
        # 绘制 RRT/RRT* 树（如果正在运行算法或已完成优化）
        elif game_state in [GameState.RUNNING_RRT, GameState.RUNNING_RRT_STAR, GameState.PATH_FOUND, GameState.OPTIMIZING_PATH, 
                          GameState.PATH_OPTIMIZED, GameState.QUIT] or algorithm_type in ['rrt', 'rrtstar']:
            # 统一使用RRT*的蓝色作为节点颜色
            tree_color = BLUE
            
            # 安全地绘制树结构
            for node in parent_map:
                parent = parent_map.get(node)
                if parent is not None:
                    # 确保节点和父节点是有效的坐标点
                    if (isinstance(node, tuple) and len(node) == 2 and 
                        isinstance(parent, tuple) and len(parent) == 2):
                        pygame.draw.line(screen, tree_color, node, parent, 1)
                        pygame.draw.circle(screen, tree_color, node, NODE_RADIUS)
            
            # 如果找到路径，绘制路径（在优化中和优化完成后都显示）
            if (game_state in [GameState.OPTIMIZING_PATH, GameState.PATH_OPTIMIZED, GameState.QUIT] 
               and end_node and end_node in parent_map):
                current = end_node
                # 安全地绘制路径
                while current != start_node and current is not None:
                    parent = parent_map.get(current)
                    if parent and parent != current:
                        # 确保节点和父节点是有效的坐标点
                        if (isinstance(current, tuple) and len(current) == 2 and 
                            isinstance(parent, tuple) and len(parent) == 2):
                            pygame.draw.line(screen, PATH_COLOR, current, parent, 3)
                        current = parent
                    else:
                        break
                         
            # 绘制椭圆约束区域（只在RRT*算法的优化中和优化完成后显示）
            if game_state in [GameState.OPTIMIZING_PATH, GameState.PATH_OPTIMIZED] and algorithm_type == 'rrtstar' and start_node and end_node and end_node in cost_map:
                try:
                    # 获取椭圆参数
                    major_axis_length = cost_map[end_node]
                    focus1 = start_node
                    focus2 = end_node
                    
                    # 计算椭圆的相关参数
                    c = math.dist(focus1, focus2) / 2
                    if major_axis_length < 2 * c:
                        major_axis_length = 2 * c
                    a = major_axis_length / 2
                    b = (a**2 - c**2)**0.5 if a > c else 0
                    
                    # 计算椭圆中心
                    center = ((focus1[0] + focus2[0]) / 2, (focus1[1] + focus2[1]) / 2)
                    
                    # 计算焦点连线的角度
                    dx = focus2[0] - focus1[0]
                    dy = focus2[1] - focus1[1]
                    angle = math.atan2(dy, dx) * 180 / math.pi  # 转换为度数
                    
                    # 创建一个临时表面来绘制旋转的椭圆
                    temp_surface = pygame.Surface((major_axis_length * 2, major_axis_length * 2), pygame.SRCALPHA)
                    
                    # 在临时表面上绘制椭圆
                    ellipse_rect = pygame.Rect(0, 0, 2*a, 2*b)
                    ellipse_rect.center = (temp_surface.get_width() // 2, temp_surface.get_height() // 2)
                    pygame.draw.ellipse(temp_surface, (0, 255, 255, 50), ellipse_rect)  # 半透明青色
                    pygame.draw.ellipse(temp_surface, (0, 150, 255), ellipse_rect, 2)  # 深蓝色边框
                    
                    # 旋转临时表面
                    rotated_surface = pygame.transform.rotate(temp_surface, -angle)
                    rotated_rect = rotated_surface.get_rect(center=center)
                    
                    # 将旋转后的椭圆绘制到主屏幕上
                    screen.blit(rotated_surface, rotated_rect.topleft)
                    
                    # 绘制焦点连线
                    pygame.draw.line(screen, (0, 150, 255), focus1, focus2, 2)
                except Exception as e:
                    print(f"Error drawing ellipse: {e}")
    except Exception as e:
        print(f"Error in redraw_scene: {e}")

# 绘制UI元素
def draw_ui(screen, font, mode_button, astar_button, rrt_button, rrtstar_button, status_text, game_state):
    """
    绘制所有UI元素
    :param screen: pygame 的屏幕对象
    :param font: pygame 的字体对象
    :param mode_button: 模式按钮对象
    :param astar_button: A*算法按钮对象
    :param rrtstar_button: RRT*算法按钮对象
    :param status_text: 状态文本内容
    :param game_state: 当前游戏状态
    """
    try:
        # 绘制模式按钮和按钮文本
        mode_button.draw(screen)
        # 渲染按钮文本
        btn_text_surf = font.render('CLICK HERE', True, WHITE)
        # 获取文本矩形并居中显示在按钮上
        btn_text_rect = btn_text_surf.get_rect(center=mode_button.rect.center)
        screen.blit(btn_text_surf, btn_text_rect)
        
        # 始终绘制算法选择按钮，但只在算法选择状态下响应点击
        astar_button.draw(screen)
        rrt_button.draw(screen)  # 绘制RRT按钮
        rrtstar_button.draw(screen)
        
        # 渲染算法按钮文本
        astar_text_surf = font.render('A*', True, WHITE)
        rrt_text_surf = font.render('RRT', True, WHITE)
        rrtstar_text_surf = font.render('RRT*', True, WHITE)
        
        astar_text_rect = astar_text_surf.get_rect(center=astar_button.rect.center)
        rrt_text_rect = rrt_text_surf.get_rect(center=rrt_button.rect.center)
        rrtstar_text_rect = rrtstar_text_surf.get_rect(center=rrtstar_button.rect.center)
        
        screen.blit(astar_text_surf, astar_text_rect)
        screen.blit(rrt_text_surf, rrt_text_rect)  # 绘制RRT按钮文本
        screen.blit(rrtstar_text_surf, rrtstar_text_rect)
        
        # 绘制状态描述文本（放在按钮上方居中位置，向下移动20像素）
        # 先用白色矩形覆盖旧文本区域，实现刷新效果
        pygame.draw.rect(screen, WHITE, (SCREEN_WIDTH // 2 - 200, SCREEN_HEIGHT - BUTTON_HEIGHT - 50, 400, 30))
        # 渲染状态文本
        status_surf = font.render(status_text, True, BLACK)
        # 获取文本矩形并设置位置（在按钮上方居中）
        status_rect = status_surf.get_rect(center=(SCREEN_WIDTH // 2, SCREEN_HEIGHT - BUTTON_HEIGHT - 35))
        screen.blit(status_surf, status_rect)
    except Exception as e:
        print(f"Error in draw_ui: {e}")