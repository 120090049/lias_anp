import csv
import numpy as np

class SonarDataReader:
    def __init__(self, filepath):
        self.filepath = filepath
        self.data = []

    def read_data(self):
        with open(self.filepath, 'r') as file:
            reader = csv.reader(file)
            # headers = next(reader)  # 跳过表头

            for row in reader:
                pose_x = float(row[0])
                pose_y = float(row[1])
                pose_z = float(row[2])
                pose_orient_x = float(row[3])
                pose_orient_y = float(row[4])
                pose_orient_z = float(row[5])
                pose_orient_w = float(row[6])

                w_p = np.array(eval(row[7]))
                s_p = np.array(eval(row[8]))
                si_q = np.array(eval(row[9]))
                si_q_theta_Rho = np.array(eval(row[10]))
                timestep = int(row[11])
                pts_indice = np.array(eval(row[12]))

                self.data.append({
                    'pose': {
                        'position': {'x': pose_x, 'y': pose_y, 'z': pose_z},
                        'orientation': {'x': pose_orient_x, 'y': pose_orient_y, 'z': pose_orient_z, 'w': pose_orient_w}
                    },
                    'w_p': w_p,
                    's_p': s_p,
                    'si_q': si_q,
                    'si_q_theta_Rho': si_q_theta_Rho,
                    'timestep': timestep,
                    'pts_indice': pts_indice
                })

    def get_data(self):
        return self.data

if __name__ == "__main__":
    filepath = "sonar_data.csv"
    reader = SonarDataReader(filepath)
    reader.read_data()
    data = reader.get_data()

    # 测试打印读取的数据
    for entry in data:
        print("Pose Position: ", entry['pose']['position'])
        print("Pose Orientation: ", entry['pose']['orientation'])
        # print("w_p: ", entry['w_p'])
        # print("s_p: ", entry['s_p'])
        # print("si_q: ", entry['si_q'])
        # print("si_q_theta_d: ", entry['si_q_theta_d'])
        # print("Timestep: ", entry['timestep'])
        # print("Pts Indice: ", entry['pts_indice'])
        # print("\n")
        break