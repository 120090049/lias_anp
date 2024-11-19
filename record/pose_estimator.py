import numpy as np
from scipy.spatial.transform import Rotation as R

class PosePredictor:
    def __init__(self):
        self.pose_history = []
        self.velocity_history = []
        self.acceleration_history = []
        self.angular_velocity_history = []
        self.angular_acceleration_history = []
        
    def add_pose(self, pose_matrix):
        """添加新的位姿到历史记录"""
        self.pose_history.append(pose_matrix)
        
        # 计算位置的速度和加速度
        if len(self.pose_history) >= 2:
            curr_pos = pose_matrix[:3, 3]
            prev_pos = self.pose_history[-2][:3, 3]
            velocity = curr_pos - prev_pos
            self.velocity_history.append(velocity)
            
            if len(self.velocity_history) >= 2:
                acceleration = self.velocity_history[-1] - self.velocity_history[-2]
                self.acceleration_history.append(acceleration)
        
        # 计算旋转的角速度和角加速度
        if len(self.pose_history) >= 2:
            curr_rot = pose_matrix[:3, :3]
            prev_rot = self.pose_history[-2][:3, :3]
            
            # 计算旋转增量
            delta_rot_mat = curr_rot @ prev_rot.T
            delta_rot = R.from_matrix(delta_rot_mat)
            rot_vec = delta_rot.as_rotvec()  # 旋转向量（角速度）
            self.angular_velocity_history.append(rot_vec)
            
            if len(self.angular_velocity_history) >= 2:
                angular_acceleration = self.angular_velocity_history[-1] - self.angular_velocity_history[-2]
                self.angular_acceleration_history.append(angular_acceleration)
        
    def predict_next_pose(self):
        """预测下一时刻的位姿"""
        if len(self.pose_history) < 3 or len(self.angular_acceleration_history) == 0:
            return None
        
        # 预测位置
        last_pos = self.pose_history[-1][:3, 3]
        last_vel = self.velocity_history[-1]
        last_acc = self.acceleration_history[-1]
        predicted_pos = last_pos + last_vel + 0.5 * last_acc
        
        # 预测旋转
        last_rot = self.pose_history[-1][:3, :3]
        last_rot_R = R.from_matrix(last_rot)
        last_ang_vel = self.angular_velocity_history[-1]
        last_ang_acc = self.angular_acceleration_history[-1]
        
        # 预测总的角增量（假设时间间隔为1）
        total_ang_increment = last_ang_vel + 0.5 * last_ang_acc
        delta_rot = R.from_rotvec(total_ang_increment)
        
        # 应用旋转增量
        predicted_rot_R = delta_rot * last_rot_R
        predicted_rot = predicted_rot_R.as_matrix()
        
        # 构建预测的4x4变换矩阵
        predicted_pose = np.eye(4)
        predicted_pose[:3, :3] = predicted_rot
        predicted_pose[:3, 3] = predicted_pos
        
        return predicted_pose
    
    def is_pose_reasonable(self, pose_matrix, position_threshold=0.02, rotation_threshold=0.02):
        """判断输入的位姿是否合理"""
        if len(self.pose_history) < 3:
            return True, None  # 无法判断，认为合理
                
        predicted_pose = self.predict_next_pose()
        if predicted_pose is None:
            return True, None
        
        # 计算位置误差
        position_error = np.linalg.norm(pose_matrix[:3, 3] - predicted_pose[:3, 3])
        
        # 计算旋转误差（以弧度表示的角度差）
        predicted_rot_R = R.from_matrix(predicted_pose[:3, :3])
        test_rot_R = R.from_matrix(pose_matrix[:3, :3])
        delta_rot_R = predicted_rot_R.inv() * test_rot_R
        rot_error = delta_rot_R.magnitude()  # 角度差
        
        # 判断是否在阈值内
        is_position_reasonable = position_error < position_threshold
        is_rotation_reasonable = rot_error < rotation_threshold
        
        # return position_error, rot_error
        is_reasonable = is_position_reasonable and is_rotation_reasonable
        print(position_error, rot_error)
        return is_reasonable, predicted_pose
    
    # 为了兼容之前的输出，我们可以返回误差值
        # return position_error, rot_error

# 使用示例
if __name__ == "__main__":
    predictor = PosePredictor()
    
    # 创建示例位姿
    pose1 = np.eye(4)  # 初始位置在原点
    
    pose2 = np.eye(4)
    pose2[:3, 3] = np.array([0.1, 0.1, 0])  # 稍微移动一点
    angle = np.deg2rad(10)
    rotation_axis = np.array([0, 0, 1])
    rot = R.from_rotvec(angle * rotation_axis)
    pose2[:3, :3] = rot.as_matrix()
    
    pose3 = np.eye(4)
    pose3[:3, 3] = np.array([0.2, 0.2, 0])  # 继续移动
    angle = np.deg2rad(20)
    rot = R.from_rotvec(angle * rotation_axis)
    pose3[:3, :3] = rot.as_matrix()
    
    # 添加历史位姿
    predictor.add_pose(pose1)
    predictor.add_pose(pose2)
    predictor.add_pose(pose3)
    
    # 预测下一个位姿
    predicted_pose = predictor.predict_next_pose()
    print("预测的下一个位姿:\n", predicted_pose)
    
    # 测试一个新的位姿是否合理
    test_pose = np.eye(4)
    test_pose[:3, 3] = np.array([0.3, 0.3, 0])  # 符合移动趋势的位姿
    angle = np.deg2rad(30)
    rot = R.from_rotvec(angle * rotation_axis)
    test_pose[:3, :3] = rot.as_matrix()
    is_reasonable = predictor.is_pose_reasonable(test_pose)
    print("\n测试位姿是否合理:", is_reasonable)
    
    # 测试一个不合理的位姿
    unreasonable_pose = np.eye(4)
    unreasonable_pose[:3, 3] = np.array([1.0, 1.0, 0])  # 移动太远，可能不合理
    angle = np.deg2rad(90)
    rot = R.from_rotvec(angle * rotation_axis)
    unreasonable_pose[:3, :3] = rot.as_matrix()
    is_reasonable = predictor.is_pose_reasonable(unreasonable_pose)
    print("不合理位姿的测试结果:", is_reasonable)