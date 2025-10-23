# -*- coding: utf-8 -*-

# 导入必要的库
from utils import get_distance, is_collision_free, is_point_in_game_area

# RRT* 核心算法
def run_rrt_star_step(parent_map, cost_map, target_point, step_size, rewire_radius, obstacles, start_node, end_node):
    """
    执行单步 RRT* 扩展算法
    :param parent_map: 存储树结构的字典，键为节点坐标，值为其父节点坐标
    :param cost_map: 存储节点成本的字典，键为节点坐标，值为从起点到该节点的成本
    :param target_point: 随机采样点 (x, y)
    :param step_size: 扩展步长
    :param rewire_radius: 重连半径
    :param obstacles: 障碍物列表
    :param start_node: 起点坐标
    :param end_node: 终点坐标
    :return: (is_success, new_node) 是否成功扩展及新节点坐标
    """
    try:
        # 1. 寻找最近的节点
        # 在现有树中找到距离目标点最近的节点
        nearest_node = min(parent_map.keys(), key=lambda p: get_distance(p, target_point))

        # 2. 计算朝向目标点的方向向量并归一化
        # 计算从最近节点指向目标点的方向向量
        direction = (target_point[0] - nearest_node[0], target_point[1] - nearest_node[1])
        # 计算最近节点与目标点之间的距离
        dist = get_distance(target_point, nearest_node)
        # 如果距离为 0，说明两点重合，无法扩展，返回失败
        if dist == 0:
            return False, None

        # 3. 生成新节点
        # 沿着方向向量，从最近节点开始，按照步长生成新节点
        new_node = (int(nearest_node[0] + direction[0] / dist * step_size),
                    int(nearest_node[1] + direction[1] / dist * step_size))

        # 检查新节点是否在游戏区域内以及是否已经在树中
        # 如果新节点不在游戏区域内或已经在树中，则此次扩展失败
        if not is_point_in_game_area(new_node[0], new_node[1]) or new_node in parent_map:
            return False, None

        # 4. 在邻域内为新节点选择最佳父节点
        # 找到所有在重连半径内的邻近节点
        neighbors = [node for node in parent_map if get_distance(node, new_node) < rewire_radius]
        
        # 如果邻域为空，则使用最近的节点作为候选
        if not neighbors:
            neighbors.append(nearest_node)
            
        # 初始化最佳父节点为最近节点，最小成本为从最近节点到达新节点的总成本
        best_parent = nearest_node
        min_cost = cost_map[nearest_node] + get_distance(nearest_node, new_node)

        # 遍历所有邻近节点，寻找成本最低的父节点
        for neighbor in neighbors:
            # 计算从该邻近节点到达新节点的总成本
            cost = cost_map[neighbor] + get_distance(neighbor, new_node)
            # 如果成本更低且路径无碰撞，则更新最佳父节点
            if cost < min_cost and is_collision_free(neighbor, new_node, obstacles):
                min_cost = cost
                best_parent = neighbor
                
        # 如果从最佳父节点到新节点的路径有障碍，则此次扩展失败
        if not is_collision_free(best_parent, new_node, obstacles):
            return False, None

        # 5. 将新节点添加到树中
        # 设置新节点的父节点
        parent_map[new_node] = best_parent
        # 记录新节点的成本
        cost_map[new_node] = min_cost

        # 6. Rewire: 重连邻域内的节点
        # 遍历所有邻近节点，尝试通过新节点优化它们的路径
        for neighbor in neighbors:
            # 不重连新节点的父节点（避免无效操作）
            if neighbor == best_parent:
                continue
            
            # 计算通过新节点到达邻近节点的新潜在成本
            new_potential_cost = cost_map[new_node] + get_distance(new_node, neighbor)
            # 如果新潜在成本更低且路径无碰撞，则进行重连
            if new_potential_cost < cost_map[neighbor] and is_collision_free(new_node, neighbor, obstacles):
                # 更新邻近节点的父节点为新节点
                parent_map[neighbor] = new_node
                # 更新邻近节点的成本
                cost_map[neighbor] = new_potential_cost
                # 更新所有依赖于该节点的后续节点的成本
                update_descendant_costs(neighbor, parent_map, cost_map, obstacles)
        
        # 扩展成功，返回新节点
        return True, new_node
    except Exception as e:
        print(f"Error in run_rrt_star_step: {e}")
        return False, None

def update_descendant_costs(node, parent_map, cost_map, obstacles):
    """
    递归更新所有依赖于指定节点的后续节点的成本
    :param node: 已更新成本的节点
    :param parent_map: 存储树结构的字典
    :param cost_map: 存储节点成本的字典
    :param obstacles: 障碍物列表
    """
    try:
        # 找到所有以该节点为父节点的子节点
        children = [child for child in parent_map if parent_map[child] == node]
        
        # 遍历所有子节点
        for child in children:
            # 检查从当前节点到子节点的路径是否无碰撞
            if is_collision_free(node, child, obstacles):
                # 计算通过当前节点到达子节点的新成本
                new_cost = cost_map[node] + get_distance(node, child)
                
                # 如果新成本更低，则更新子节点的成本
                if new_cost < cost_map.get(child, float('inf')):
                    cost_map[child] = new_cost
                    # 递归更新该子节点的所有后续节点的成本
                    update_descendant_costs(child, parent_map, cost_map, obstacles)
    except Exception as e:
        print(f"Error in update_descendant_costs: {e}")