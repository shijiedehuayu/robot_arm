import numpy as np
from typing import List

class CubicSplineInterpolation:
    def __init__(self, dt: float):
        """
        单维度三次样条插值器初始化
        
        参数:
            dt: 时间步长 (秒)
        """
        self.dt = dt
    
    
    def interpolate_with_details(self, start_pos: float, end_pos: float, 
                                total_time: float, 
                                start_vel: float = 0.0, 
                                end_vel: float = 0.0) -> List[tuple]:
        """
        生成详细的轨迹信息
        
        返回:
            列表，每个元素为 (时间, 位置, 速度, 加速度) 的元组
        """
        # 计算系数
        T = total_time
        a = start_pos
        b = start_vel
        c = (3 * (end_pos - start_pos) - (2 * start_vel + end_vel) * T) / (T ** 2)
        d = (2 * (start_pos - end_pos) + (start_vel + end_vel) * T) / (T ** 3)
        
        # 生成轨迹
        trajectory = []
        n_steps = int(total_time / self.dt) + 1
        
        for i in range(n_steps):
            t = i * self.dt
            if t > total_time:
                t = total_time
            
            # 计算位置、速度、加速度
            position = a + b * t + c * (t ** 2) + d * (t ** 3)
            velocity = b + 2 * c * t + 3 * d * (t ** 2)
            acceleration = 2 * c + 6 * d * t
            
            trajectory.append((t, position, velocity, acceleration))
        
        # 确保最后一个点正好是终点
        t, _, _, _ = trajectory[-1]
        trajectory[-1] = (t, end_pos, end_vel, 2 * c + 6 * d * T)
        
        return trajectory


# 使用示例
if __name__ == "__main__":
    # 初始化插值器，时间步长为0.1秒
    spline = CubicSplineInterpolation(dt=0.1)

    
    # 示例2: 获取详细轨迹信息
    detailed_trajectory = spline.interpolate_with_details(
        start_pos=0.0,
        end_pos=5.0,
        total_time=2.0,
        start_vel=1.0,  # 起始速度不为0
        end_vel=-1.0    # 终止速度为负
    )
    
    print("详细轨迹信息 (时间, 位置, 速度, 加速度):")
    for i, (t, pos, vel, acc) in enumerate(detailed_trajectory):
        if i % 5 == 0 or i == len(detailed_trajectory) - 1:  # 每5个点输出一次
            print(f"t={t:.2f}s: 位置={pos:.3f}, 速度={vel:.3f}, 加速度={acc:.3f}")