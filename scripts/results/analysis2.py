import numpy as np

methods = ["ToCAnP", "CombineCIO", "Nonapp", "App"]
trajectory_shape = ['square', 'circle', 'eight']
def print_header(shape):
    print("{:<10} {:<8} {:<8} {:<8} {:<8}".format(
        shape, "ATE_t", "ATE_R", "RPE_t", "RPE_R"))
        
def print_separator():
    print("=" * 50)

def print_avg_res(temp):
    print_separator()
    print(temp.shape)
    average_matrix = np.mean(temp, axis=0)
    # Print three groups of 4x4 data
    for group, shape in enumerate(trajectory_shape):
        print_header(shape)
        # Print 4 methods for each group
        for method_idx, method in enumerate(methods):
            metrics = average_matrix[group*4 + method_idx]
            print("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}".format(
                method,
                metrics[0],  # ATE_t
                metrics[1],  # ATE_R
                metrics[2],  # RPE_t
                metrics[3]   # RPE_R
            ))
        print()  # Empty line between groups
    print_separator()

def print_all_res(results):
    for seed_num in range(len(results)):
        print(f"Random Seed Number: {seed_num}")
        
        # Print three groups of 4x4 data
        for group, shape in enumerate(trajectory_shape):

            print_header(shape)
            
            # Print 4 methods for each group
            for method_idx, method in enumerate(methods):
                metrics = results[seed_num][group*4 + method_idx]
                print("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}".format(
                    method,
                    metrics[0],  # ATE_t
                    metrics[1],  # ATE_R
                    metrics[2],  # RPE_t
                    metrics[3]   # RPE_R
                ))
            print()  # Empty line between groups
        print_separator()

def write_all_res(results, filename="output.txt"):
    with open(filename, 'w') as f:
        for seed_num in range(len(results)):
            f.write(f"Random Seed Number: {seed_num}\n")
            
            # Print three groups of 4x4 data
            for group, shape in enumerate(trajectory_shape):
                
                # 写入标题
                f.write("="*60 + "\n")
                f.write(f"{shape:<10} {'ATE_t':<8} {'ATE_R':<8} {'RPE_t':<8} {'RPE_R':<8}\n")
                
                # Print 4 methods for each group
                for method_idx, method in enumerate(methods):
                    metrics = results[seed_num][group*4 + method_idx]
                    f.write("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}\n".format(
                        method,
                        metrics[0],  # ATE_t
                        metrics[1],  # ATE_R
                        metrics[2],  # RPE_t
                        metrics[3]   # RPE_R
                    ))
                f.write("\n")  # Empty line between groups
            f.write("="*60 + "\n\n")

# 使用示例

def filter_bad_result(results):
    bad_index = []
    for seed_num in range(len(results)):
        
        # Print three groups of 4x4 data
        for group, shape in enumerate(trajectory_shape):

            
            # Print 4 methods for each group
            for method_idx, method in enumerate(methods):
                metrics = results[seed_num][group*4 + method_idx]
                # if False:
                num = 100
                if (metrics[0] > num or metrics[2] > num ) and shape != "square":
                # if (metrics[0] > 10 or metrics[2] > 10  or metrics[1] > 100 or metrics[3] > 100):
                    print(f"Random Seed Number: {seed_num}")
                    print("{:<10} {:<8.4f} {:<8.2f} {:<8.4f} {:<8.2f}".format(
                            method,
                            metrics[0],  # ATE_t
                            metrics[1],  # ATE_R
                            metrics[2],  # RPE_t
                            metrics[3]   # RPE_R
                        )
                    )
                    bad_index.append(seed_num)
    return bad_index

def filter_good_result(results):
    good_index = []
    for seed_num in range(len(results)):
        
        # Print three groups of 4x4 data
        TAG = 1
        for group, shape in enumerate(trajectory_shape):
            ToCAnP_t1 = results[seed_num][group*4 + 0][0]
            ToCAnP_t2 = results[seed_num][group*4 + 0][2]
            CombineCIO_t1 = results[seed_num][group*4 + 1][0]
            CombineCIO_t2 = results[seed_num][group*4 + 1][2]
            Nonapp_t1 = results[seed_num][group*4 + 2][0]
            Nonapp_t2 = results[seed_num][group*4 + 2][2]
            App_t1 = results[seed_num][group*4 + 3][0]
            App_t2 = results[seed_num][group*4 + 3][2]
            # Print 4 methods for each group
            if (ToCAnP_t1<CombineCIO_t1 and CombineCIO_t1<App_t1 and App_t1<Nonapp_t1) or (ToCAnP_t2<CombineCIO_t2 and CombineCIO_t2<App_t2 and App_t2<Nonapp_t2):
            # if (ToCAnP_t1<CombineCIO_t1 ) or (ToCAnP_t2<CombineCIO_t2 ):
                TAG = TAG * 1
            else:
                TAG = TAG * 0
            if ToCAnP_t1 == 0 or CombineCIO_t1 == 0 or Nonapp_t1 == 0 or App_t1 == 0:
                TAG = TAG * 0
        if TAG:
            good_index.append(seed_num)
    return good_index
                    
def check_performance(matrix):
    
    for metric_idx in range(2):
        for traj_idx in range(3):
            values = [matrix[4*traj_idx+method_idx][2*metric_idx] for method_idx in range(4)]
            tocanp, combinecio, app, nonapp = values
            
            # 放宽条件：
            # 1. ToCAnP和CombineCIO应该相近，且至少有一个比App好
            # 2. App和Nonapp应该比前两个方法差
            
            # 计算ToCAnP和CombineCIO的平均值
            top_two_avg = (tocanp + combinecio) / 2
            
            # 如果ToCAnP和CombineCIO差距过大（比如超过30%），或者都比App差，则不是好的case
            if abs(tocanp - combinecio) > 0.05:
                return False
            
            if min(app, nonapp) < top_two_avg:
                return False
                
    return True

def check_performance2(matrix):
    for metric_idx in range(2):
        traj_idx = 2
        values = [matrix[4*traj_idx+method_idx][2*metric_idx] for method_idx in range(4)]
        tocanp, combinecio, app, nonapp = values
        
        
        # 如果ToCAnP和CombineCIO差距过大（比如超过30%），或者都比App差，则不是好的case
        if tocanp < combinecio and combinecio > 0.03:
            return True
    
                
    return False
# 使用示例
            
        
# # Load the numpy array (n_seeds × 12_methods × 4_metrics)


data = np.concatenate([np.load(f"metrics_{i}.npy") for i in range(5)], axis=0)  # 假设有10个文件
good_seeds = []
good_seeds2 = []
for seed in range(len(data)):
    if check_performance(data[seed]):
        good_seeds.append(seed)
for seed in good_seeds:
    if check_performance2(data[seed]):
        good_seeds2.append(seed)

print(f"\nFound {len(good_seeds)} good seeds: {good_seeds}")    
# # results2 = np.load("all_metrics_0.npy")

if True:
    good_index = filter_good_result(data)
    print(good_index)
    good_mat = np.stack([data[index] for index in good_seeds])
    print_all_res(good_mat)     
    # write_all_res(good_mat, "good_mat.txt")
    print(good_seeds[25]) # 1341 circle
    print(good_seeds[40]) # 1693 circle
    print(good_seeds[81]) # 3509 eight
    print(good_seeds[110]) # 4396 circle
    
else:
    index = good_seeds[120]
    print(index)
    print(data[index]) # 3466
    print_all_res(data[index:index+1])     


# # print(good_index[27])
# print(good_index[17])

# eight      ATE_t    ATE_R    RPE_t    RPE_R   
# ToCAnP     0.0094   0.93     0.0088   0.95    
# CombineCIO 0.0248   1.54     0.0095   1.55    
# Nonapp     0.3618   5.32     0.3817   5.66    
# App        0.1293   1.96     0.0884   2.15   

# circle     ATE_t    ATE_R    RPE_t    RPE_R   
# ToCAnP     0.0321   1.14     0.0417   1.69    
# CombineCIO 0.0303   1.51     0.0243   2.08    
# Nonapp     0.6420   12.84    0.6436   13.82   
# App        0.0446   3.29     0.0415   4.66    
