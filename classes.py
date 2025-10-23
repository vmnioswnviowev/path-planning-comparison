# -*- coding: utf-8 -*-

# 导入必要的库
from enum import Enum
import pygame
from constants import BUTTON_WIDTH, BUTTON_HEIGHT, BLACK

# 游戏状态枚举定义
# 使用枚举类型来表示游戏的不同状态，提高代码可读性和维护性
class GameState(Enum):
    INIT = 0                # 初始化状态
    DRAW_OBSTACLES = 1      # 绘制障碍物状态
    SET_START = 2           # 设置起点状态
    SET_END = 3             # 设置终点状态
    SELECT_ALGORITHM = 4    # 选择算法状态
    RUNNING_ASTAR = 5       # 运行 A* 算法状态
    RUNNING_RRT = 6         # 运行 RRT 算法状态
    RUNNING_RRT_STAR = 7    # 运行 RRT* 算法状态
    PATH_FOUND = 8          # 找到路径状态
    OPTIMIZING_PATH = 9     # 路径优化状态
    PATH_OPTIMIZED = 10     # 路径优化完成状态
    QUIT = 11               # 退出状态

class Button:
    """一个简单的按钮类，用于处理 UI 中的按钮操作"""
    
    def __init__(self, x, y, width, height, color):
        """
        初始化按钮对象
        :param x: 按钮左上角 x 坐标
        :param y: 按钮左上角 y 坐标
        :param width: 按钮宽度
        :param height: 按钮高度
        :param color: 按钮颜色 (RGB 格式)
        """
        # 创建一个矩形对象来表示按钮的位置和大小
        self.rect = pygame.Rect(x, y, width, height)
        # 设置按钮的颜色
        self.color = color

    def draw(self, screen):
        """
        在屏幕上绘制按钮
        :param screen: pygame 的屏幕对象
        """
        # 使用 pygame.draw.rect 绘制矩形按钮
        pygame.draw.rect(screen, self.color, self.rect)

    def is_clicked(self, pos):
        """
        检查按钮是否被点击
        :param pos: 鼠标点击位置 (x, y)
        :return: 如果点击位置在按钮矩形内返回 True，否则返回 False
        """
        # 使用 collidepoint 方法检查点击位置是否在按钮矩形内
        return self.rect.collidepoint(pos)