import numpy as np

R_matrix = np.eye(3)

# Extracting the rows
row1 = R_matrix[0, :]
row2 = R_matrix[1, :]
row3 = R_matrix[2, :]

print("Row 1:", row1)
print("Row 2:", row2)
print("Row 3:", row3)
