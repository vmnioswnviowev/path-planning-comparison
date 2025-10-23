import pandas as pd
import os
import pandas as pd

class ExcelLogger:
    def __init__(self, file_path=None):
        """初始化ExcelLogger类
        
        Args:
            file_path: Excel文件保存路径，如果为None则使用固定文件名
        """
        if file_path is None:
            # 使用固定文件名
            file_path = "algorithm_results.xlsx"
        self.file_path = file_path
        
        # 初始化数据存储结构
        self.results = {
            'astar': [],
            'rrt': [],
            'rrtstar': []
        }
        
        # 检查文件是否存在，如果存在则读取已有数据
        if os.path.exists(self.file_path):
            try:
                existing_df = pd.read_excel(self.file_path)
                # 按算法类型整理数据
                for algo in self.results.keys():
                    algo_data = existing_df[existing_df['algorithm'] == algo]
                    if not algo_data.empty:
                        self.results[algo] = algo_data.to_dict('records')
            except Exception as e:
                print(f"读取现有Excel文件失败: {e}")
                # 如果读取失败，从头开始记录
                pass
    
    def log_astar_result(self, path_length, time_taken):
        """记录A*算法结果
        
        Args:
            path_length: 路径长度
            time_taken: 算法耗时(秒)
        """
        record = {
            'algorithm': 'astar',
            'path_length': path_length,
            'time_taken': time_taken
        }
        self.results['astar'].append(record)
        print(f"已记录A*算法结果: 路径长度={path_length:.2f}, 耗时={time_taken:.2f}秒")
    
    def log_rrt_result(self, path_length, time_taken):
        """记录RRT算法结果
        
        Args:
            path_length: 路径长度
            time_taken: 算法耗时(秒)
        """
        record = {
            'algorithm': 'rrt',
            'path_length': path_length,
            'time_taken': time_taken
        }
        self.results['rrt'].append(record)
        print(f"已记录RRT算法结果: 路径长度={path_length:.2f}, 耗时={time_taken:.2f}秒")
    
    def log_rrtstar_result(self, path_length, time_taken, improvement_percentage=None):
        """记录RRT*算法结果
        
        Args:
            path_length: 路径长度
            time_taken: 算法耗时(秒)
            improvement_percentage: 路径改善百分比(可选)
        """
        record = {
            'algorithm': 'rrtstar',
            'path_length': path_length,
            'time_taken': time_taken,
            'improvement_percentage': improvement_percentage
        }
        self.results['rrtstar'].append(record)
        if improvement_percentage is not None:
            print(f"已记录RRT*算法结果: 路径长度={path_length:.2f}, 耗时={time_taken:.2f}秒, 改善百分比={improvement_percentage:.2f}%")
        else:
            print(f"已记录RRT*算法结果: 路径长度={path_length:.2f}, 耗时={time_taken:.2f}秒")
    
    def log_point_optimization(self, algorithm, original_points_count, optimized_points_count, original_path_length, optimized_path_length):
        """记录路径点优化结果
        
        Args:
            algorithm: 算法名称
            original_points_count: 原始路径点数量
            optimized_points_count: 优化后路径点数量
            original_path_length: 原始路径长度
            optimized_path_length: 优化后路径长度
        """
        # 确保算法类型有效
        if algorithm not in self.results:
            print(f"警告: 未知算法类型 '{algorithm}'，优化结果未记录")
            return
        
        # 在最近的记录中添加优化信息
        if self.results[algorithm]:
            last_record = self.results[algorithm][-1]
            last_record.update({
                'original_points_count': original_points_count,
                'optimized_points_count': optimized_points_count,
                'original_path_length': original_path_length,
                'optimized_path_length': optimized_path_length
            })
            point_reduction = ((original_points_count - optimized_points_count) / original_points_count) * 100 if original_points_count > 0 else 0
            print(f"已记录{algorithm.upper()}算法路径点优化结果: 原始点数={original_points_count}, 优化后点数={optimized_points_count}, 点数减少={point_reduction:.2f}%")
        else:
            print(f"警告: 没有找到{algorithm.upper()}算法的记录，优化结果未记录")
    
    def save_to_excel(self):
        """将所有记录保存到Excel文件"""
        try:
            # 合并所有算法的结果
            all_results = []
            for algo_results in self.results.values():
                all_results.extend(algo_results)
            
            if not all_results:
                print("没有数据可保存到Excel")
                return
            
            # 创建DataFrame
            df = pd.DataFrame(all_results)
            
            # 保存到Excel
            df.to_excel(self.file_path, index=False)
            print(f"所有结果已成功保存到Excel文件: {self.file_path}")
        except Exception as e:
            print(f"保存到Excel文件失败: {e}")

# 创建一个全局的ExcelLogger实例，方便在main.py中使用
excel_logger = ExcelLogger()