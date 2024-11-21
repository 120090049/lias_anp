import numpy as np

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

def filter_result(results):
    bad_index = []
    for seed_num in range(len(results)):
        
        # Print three groups of 4x4 data
        for group, shape in enumerate(trajectory_shape):

            
            # Print 4 methods for each group
            for method_idx, method in enumerate(methods):
                metrics = results[seed_num][group*4 + method_idx]

                if (metrics[0] > 4 or metrics[2] > 4 or metrics[1] > 100 or metrics[3] > 100):
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
                    
# Load the numpy array (n_seeds × 12_methods × 4_metrics)
methods = ["ToCAnP", "CombineCIO", "Nonapp", "App"]
trajectory_shape = ['square', 'circle', 'eight']

results = np.load("all_metrics_001_.npy")
for i in range(5,10):
    print_avg_res(results[i*10:i*10+10])
    print()

# bad_index = filter_result(results)
# print()

# temp = np.delete(results, bad_index, axis=0)  # 删除bad_index对应的矩阵
# print_avg_res(temp)
# # temp = results[20:30]

