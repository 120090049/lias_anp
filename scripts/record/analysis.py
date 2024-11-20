import numpy as np

def parse_experiment_data(filename):
    data = {}
    current_seed = None
    skip_current_seed = False
    
    with open(filename, 'r') as f:
        lines = f.readlines()
    
    for line in lines:
        line = line.strip()
        if not line or '=' in line or '+' in line:
            continue
            
        if line.startswith('Random Seed Number:'):
            current_seed = int(line.split(':')[1].strip())
            data[current_seed] = np.zeros((3, 4, 4))
            trajectory_count = 0
            method_count = 0
            skip_current_seed = False  # 重置标志
            continue
            
        elif line.startswith('method'):
            method_count = 0
            continue
            
        else:
            parts = line.split()
            if len(parts) == 5 and current_seed is not None and not skip_current_seed:
                # 检查是否有大于100的值
                a = 4
                b = 80
                if float(parts[1]) > a or float(parts[2]) > b or float(parts[3]) > a or float(parts[4]) > b:
                    print(current_seed)
                    print(parts)
                    del data[current_seed]  # 删除这个seed的数据
                    skip_current_seed = True  # 设置标志以跳过这个seed的剩余数据
                    continue
                
                if not skip_current_seed:
                    data[current_seed][trajectory_count][method_count] = [
                        float(parts[1]),  # ATE_t
                        float(parts[2]),  # ATE_R
                        float(parts[3]),  # RPE_t
                        float(parts[4])   # RPE_R
                    ]
                    method_count += 1
                    if method_count == 4:  # 一组方法数据读完
                        trajectory_count += 1
    
    return data

# 使用示例
methods = ['ToCAnP', 'CombineCIO', 'Nonapp', 'App']
metrics = ['ATE_t', 'ATE_R', 'RPE_t', 'RPE_R']    
filename = '/home/clp/catkin_ws/src/lias_anp/scripts/record/0_001.txt'
data = parse_experiment_data(filename)



def calculate_average(data):
    # 计算平均值
    all_matrices = np.stack([data[seed] for seed in data])
    average_matrix = np.mean(all_matrices, axis=0)
    
    # 打印平均值
    trajectory_names = ['Circle', 'Square', 'Eight']
    methods = ['ToCAnP', 'CombineCIO', 'Nonapp', 'App']
    
    print("Average values across all random seeds:")
    for traj_idx, traj_name in enumerate(trajectory_names):
        print(f"\n{traj_name} Trajectory:")
        print("method       ATE_t    ATE_R    RPE_t    RPE_R")
        print("-" * 45)
        for method_idx, method in enumerate(methods):
            values = average_matrix[traj_idx][method_idx]
            print(f"{method:<12} {values[0]:7.4f} {values[1]:7.2f} {values[2]:7.4f} {values[3]:7.2f}")
    
    return average_matrix

# 打印平均值
trajectory_names = ['Circle', 'Square', 'Eight']
methods = ['ToCAnP', 'CombineCIO', 'Nonapp', 'App']
average_matrix = calculate_average(data)
