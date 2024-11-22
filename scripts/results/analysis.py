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
            if (ToCAnP_t1<CombineCIO_t1 and CombineCIO_t1<Nonapp_t1 and CombineCIO_t1<App_t1 and CombineCIO_t1<Nonapp_t1) or (ToCAnP_t2<CombineCIO_t2 and CombineCIO_t2<Nonapp_t2 and CombineCIO_t2<App_t2 and CombineCIO_t2<Nonapp_t2):
            # if (ToCAnP_t1<CombineCIO_t1 ) or (ToCAnP_t2<CombineCIO_t2 ):
                TAG = TAG * 1
            else:
                TAG = TAG * 0
            if ToCAnP_t1 == 0 or CombineCIO_t1 == 0 or Nonapp_t1 == 0 or App_t1 == 0:
                TAG = TAG * 0
        if TAG:
            good_index.append(seed_num)
    return good_index
                    
# Load the numpy array (n_seeds × 12_methods × 4_metrics)
methods = ["ToCAnP", "CombineCIO", "Nonapp", "App"]
trajectory_shape = ['square', 'circle', 'eight']

results = np.load("all_metrics_001_5000.npy")
print(len(results))
good_index = filter_good_result(results)
print(good_index)
good_mat = np.stack([results[index] for index in good_index])

print_all_res(good_mat[27:28])
print(good_index[27])
print(results[good_index[27]]) # 3466
# for i in range(5,10):
#     print_avg_res(results[i*10:i*10+10])
#     print()

# bad_index = filter_result(results)
# print()

# temp = np.delete(results, bad_index, axis=0)  # 删除bad_index对应的矩阵
# print_avg_res(temp)
# # # # temp = results[20:30]

# 3466
# [[1.2680e-01 3.4200e+00 4.8800e-02 3.6800e+00]
#  [7.5400e-02 3.1500e+00 8.4400e-02 3.8500e+00]
#  [1.3476e+00 2.5930e+01 1.1005e+00 1.4630e+01]
#  [8.2600e-02 2.3300e+00 1.0710e-01 2.8300e+00]
#  [3.8400e-02 1.2300e+00 2.9000e-02 1.5100e+00]
#  [1.3150e-01 1.9900e+00 3.2600e-02 2.1600e+00]
#  [3.3800e-01 3.5300e+00 3.5010e-01 3.7300e+00]
#  [1.4470e-01 3.1500e+00 1.4170e-01 4.2300e+00]
#  [7.8000e-03 1.5800e+00 8.2000e-03 1.7600e+00]
#  [2.0000e-02 8.8000e-01 9.3000e-03 9.5000e-01]
#  [3.2537e+00 6.8490e+01 4.2423e+00 8.7150e+01]
#  [1.1456e+00 1.6510e+01 9.4390e-01 1.3040e+01]]
# [98, 145, 266, 345, 559, 637, 645, 681, 894, 934, 961, 1059, 1208, 1235, 1243, 1467, 1475, 1577, 1728, 1755, 1908, 2349, 2513, 2629, 2745, 3208, 3315, 3466, 3574, 3706, 3902, 4238, 4256, 4304, 4553, 4728, 4743, 4826, 4880, 4926, 4958]