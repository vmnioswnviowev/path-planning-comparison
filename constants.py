# -*- coding: utf-8 -*-

# 定义屏幕尺寸常量
SCREEN_WIDTH = 800      # 屏幕宽度为 800 像素 - 增大了屏幕尺寸
SCREEN_HEIGHT = 650     # 屏幕高度为 650 像素 - 增大了屏幕尺寸

# 游戏区域参数定义
GAME_X = 50             # 游戏区域左上角 x 坐标 - 调整了位置
GAME_Y = 50             # 游戏区域左上角 y 坐标 - 调整了位置
GAME_WIDTH = 700        # 游戏区域宽度为 700 像素 - 增大了游戏区域
GAME_HEIGHT = 500       # 游戏区域高度为 500 像素 - 增大了游戏区域
GAME_BORDER = 3         # 游戏区域边框宽度为 3 像素

# 路径优化参数定义
MAX_OPTIMIZATION_ITERATIONS = 500   # 路径优化的最大迭代次数，防止无限循环
MAX_OPTIMIZATION_TIME = 3           # 路径优化的最大时间（秒），控制优化时长

# 颜色定义（RGB 格式）
WHITE = (255, 255, 255)     # 白色
BLACK = (0, 0, 0)           # 黑色
RED = (255, 0, 0)           # 红色
GREEN = (0, 255, 0)         # 绿色
BLUE = (0, 0, 255)          # 蓝色
YELLOW = (255, 255, 0)      # 黄色，用于A*算法和按钮
PATH_COLOR = (20, 200, 20)  # 路径颜色，更亮眼的绿色
REWIRE_LINE_COLOR = (128, 0, 128)  # 重连时的线颜色，紫色
OPEN_SET_COLOR = (255, 165, 0)  # 开放列表节点颜色，橙色
CLOSED_SET_COLOR = (128, 128, 128)  # 关闭列表节点颜色，灰色

# RRT* 算法参数定义
STEP_SIZE = 10          # 扩展步长，控制每次扩展的距离
NODE_RADIUS = 2         # 节点绘制半径（像素）
GOAL_RADIUS = 10        # 起点和终点的绘制半径（像素）
OBSTACLE_RADIUS = 10    # 障碍物绘制半径（像素）

# RRT* 特有参数
REWIRE_RADIUS = 40      # 重连半径，用于寻找邻近节点进行重新连接

# UI 元素参数定义
BUTTON_X = 50           # 按钮左上角 x 坐标 - 调整了位置
BUTTON_Y = 580          # 按钮左上角 y 坐标 - 调整了位置
BUTTON_WIDTH = 120      # 按钮宽度 - 增大了按钮尺寸
BUTTON_HEIGHT = 40      # 按钮高度 - 调整了按钮尺寸
STATUS_TEXT_POS = (200, 600)  # 状态文本位置 - 调整了位置

# 椭圆约束采样参数
ELLIPSE_FOCUS_WEIGHT = 0.5  # 椭圆焦点权重
ELLIPSE_PROBABILITY = 0.9   # 在椭圆内采样的概率

# 网格参数
GRID_SIZE = 5  # 网格大小，5x5像素