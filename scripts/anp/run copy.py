import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation
import sys
import os

# Add path to your Python scripts
py_path = 'C:/Users/Administrator/Desktop/ANP_NEW'
sys.path.append(py_path)



# Import your existing Python functions
from App_Algorithm_2 import App_Algorithm_2
from Nonapp_Algorithm_2 import Nonapp_Algorithm_2
from Combine_CIO_2 import Combine_CIO_2
from ToCAnP import ToCAnP
from Calculate_CRLB import Calculate_CRLB

# import scipy
# print(scipy.__version__)


# True rotation matrix
R_true = np.array([
    [0.689673028293550, 0.423447870703166, 0.587403621746887],
    [0.176155229057263, 0.688715772969376, -0.703306419236294],
    [-0.702367745073895, 0.588525687510876, 0.400396136119796]
])

# True translation vector
t_true = np.array([[2], [3], [4]])
t_s_true = -R_true.T @ t_true

# Number of points to test
# num = [10, 30, 90, 270, 810, 1000]
# num = [10, 30, 90, 270]

num = [30]
M = 1  # Number of Monte Carlo iterations

# Initialize arrays to store errors
Error_t_App = np.zeros(len(num))
Error_R_App = np.zeros(len(num))
Error_t_combine_CIO = np.zeros(len(num))
Error_R_combine_CIO = np.zeros(len(num))
Error_t_Nonapp = np.zeros(len(num))
Error_R_Nonapp = np.zeros(len(num))
Error_t_ToCAnP = np.zeros(len(num))
Error_R_ToCAnP = np.zeros(len(num))
CRLB_t_x = np.zeros(len(num))
CRLB_R_x = np.zeros(len(num))

for n_idx, n in enumerate(num):
    print(f"Processing {n} points")
    phi_max = 10 * np.pi / 180
    
    error_t_App = 0
    error_R_App = 0
    error_t_combine_CIO = 0
    error_R_combine_CIO = 0
    error_t_Nonapp = 0
    error_R_Nonapp = 0
    error_t_ToCAnP = 0
    error_R_ToCAnP = 0

    for j in range(M):
        # Generate random angles and distances
        elevation_angles = np.random.uniform(-phi_max, phi_max, n)
        azimuth_angles = np.random.uniform(-np.pi/6, np.pi/6, n)
        d_true = np.random.uniform(0, 6, n)

        # Generate p_s
        p_s = np.zeros((3, n))
        p_s[0] = d_true * np.cos(elevation_angles) * np.cos(azimuth_angles)
        p_s[1] = d_true * np.cos(elevation_angles) * np.sin(azimuth_angles)
        p_s[2] = d_true * np.sin(elevation_angles)

        # Generate p_si
        p_si = np.zeros((2, n))
        p_si[0] = d_true * np.cos(azimuth_angles)
        p_si[1] = d_true * np.sin(azimuth_angles)

        # Generate p_w
        p_w = R_true @ p_s + t_true

        # Add noise
        temp_std_noise = 0.001
        stdVar_noise_d = temp_std_noise
        stdVar_noise_theta = temp_std_noise

        d_noise = d_true + stdVar_noise_d * np.random.randn(n)
        noise_on_theta = stdVar_noise_theta * np.random.randn(n)

        tan_theta = np.tan(azimuth_angles)
        tan_theta_noise_ori = tan_theta + noise_on_theta
        theta_noise_tan_ori = np.arctan(tan_theta_noise_ori)

        cos_theta_noise_tan = np.cos(theta_noise_tan_ori)
        sin_theta_noise_tan = np.sin(theta_noise_tan_ori)
        p_si_noise = np.vstack((d_noise * cos_theta_noise_tan, d_noise * sin_theta_noise_tan))

        # Run algorithms
        R_app, t_app = App_Algorithm_2(p_w, p_si_noise, phi_max)
        R_Nonapp, t_Nonapp = Nonapp_Algorithm_2(p_w, p_si_noise, phi_max, R_true)
        R_combine_CIO, t_combine_CIO = Combine_CIO_2(p_w, p_si_noise, phi_max, R_true)
        R_ToCAnP, t_ToCAnP = ToCAnP(p_w, p_si_noise)

        # Calculate errors
        # For rotation errors, using scipy.spatial.transform
        def rot_error(R_est, R_true):
            R_diff = R_est @ R_true.T
            r = Rotation.from_matrix(R_diff)
            return np.linalg.norm(r.as_euler('xyz'))

        error_R_App += rot_error(R_app, R_true)**2
        error_R_combine_CIO += rot_error(R_combine_CIO, R_true)**2
        error_R_Nonapp += rot_error(R_Nonapp, R_true)**2
        error_R_ToCAnP += rot_error(R_ToCAnP, R_true)**2

        error_t_App += np.linalg.norm(t_app - t_true)**2
        error_t_combine_CIO += np.linalg.norm(t_combine_CIO - t_true)**2
        error_t_Nonapp += np.linalg.norm(t_Nonapp - t_true)**2
        error_t_ToCAnP += np.linalg.norm(t_ToCAnP - t_true)**2

    # Calculate CRLB
    crlb_R, crlb_t = Calculate_CRLB(p_w, R_true, t_true, stdVar_noise_d, stdVar_noise_theta)

    # Store results
    CRLB_t_x[n_idx] = np.sqrt(crlb_t)
    CRLB_R_x[n_idx] = np.sqrt(crlb_R)
    
    Error_t_App[n_idx] = np.sqrt(error_t_App/M)
    Error_R_App[n_idx] = np.sqrt(error_R_App/M)
    Error_t_combine_CIO[n_idx] = np.sqrt(error_t_combine_CIO/M)
    Error_R_combine_CIO[n_idx] = np.sqrt(error_R_combine_CIO/M)
    Error_t_Nonapp[n_idx] = np.sqrt(error_t_Nonapp/M)
    Error_R_Nonapp[n_idx] = np.sqrt(error_R_Nonapp/M)
    Error_t_ToCAnP[n_idx] = np.sqrt(error_t_ToCAnP/M)
    Error_R_ToCAnP[n_idx] = np.sqrt(error_R_ToCAnP/M)
    
print('R_true:',R_true)
print('R_ToCAnP:',R_ToCAnP)
print('R_Nonapp:',R_Nonapp)
print('R_app:',R_app)
print('R_combine_CIO:',R_combine_CIO)
print('t_true:',t_true)
print('t_ToCAnP:',t_ToCAnP)
print('t_Nonapp:',t_Nonapp)
print('t_app:',t_app)
print('t_combine_CIO:',t_combine_CIO)


# # Plotting
# plt.figure(figsize=(10, 8))
# plt.grid(True)
# plt.loglog(num, Error_t_App, '--', linewidth=2, label='App')
# plt.loglog(num, Error_t_Nonapp, '--', linewidth=2, label='Nonapp')
# plt.loglog(num, Error_t_combine_CIO, '-.', linewidth=2, label='Combine+CIO')
# plt.loglog(num, Error_t_ToCAnP, '--', linewidth=2, label='ToCAnP')
# plt.loglog(num, CRLB_t_x, linewidth=2, label='CRLB')
# plt.xlabel('Point Number', fontsize=12, fontweight='bold')
# plt.ylabel('RMSE_t', fontsize=12, fontweight='bold')
# plt.legend(fontsize=12)
# plt.xticks(fontsize=12)
# plt.yticks(fontsize=12)
# plt.show()



# Additional plots can be added similarly...