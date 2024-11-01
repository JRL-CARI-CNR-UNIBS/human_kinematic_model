import numpy as np
from scipy.spatial.transform import Rotation as R


class Keypoints:
    def __init__(self,
                 head           = np.zeros(3),
                 left_shoulder  = np.zeros(3),
                 left_elbow     = np.zeros(3),
                 left_wrist     = np.zeros(3),
                 left_hip       = np.zeros(3),
                 left_knee      = np.zeros(3),
                 left_ankle     = np.zeros(3),
                 right_shoulder = np.zeros(3),
                 right_elbow    = np.zeros(3),
                 right_wrist    = np.zeros(3),
                 right_hip      = np.zeros(3),
                 right_knee     = np.zeros(3),
                 right_ankle    = np.zeros(3)):
        
        self.head = head
        self.left_shoulder  = left_shoulder
        self.left_elbow     = left_elbow
        self.left_wrist     = left_wrist
        self.left_hip       = left_hip
        self.left_knee      = left_knee
        self.left_ankle     = left_ankle
        self.right_shoulder = right_shoulder
        self.right_elbow    = right_elbow
        self.right_wrist    = right_wrist
        self.right_hip      = right_hip
        self.right_knee     = right_knee
        self.right_ankle    = right_ankle


    def set_keypoints(self, keypoints: dict):
        self.head           = keypoints["head"]
        self.left_shoulder  = keypoints["left_shoulder"]
        self.left_elbow     = keypoints["left_elbow"]
        self.left_wrist     = keypoints["left_wrist"]
        self.left_hip       = keypoints["left_hip"]
        self.left_knee      = keypoints["left_knee"]
        self.left_ankle     = keypoints["left_ankle"]
        self.right_shoulder = keypoints["right_shoulder"]
        self.right_elbow    = keypoints["right_elbow"]
        self.right_wrist    = keypoints["right_wrist"]
        self.right_hip      = keypoints["right_hip"]
        self.right_knee     = keypoints["right_knee"]
        self.right_ankle    = keypoints["right_ankle"]

    
    def get_keypoints(self):
        return np.array([self.head, self.left_shoulder, self.left_elbow, self.left_wrist,
                         self.left_hip, self.left_knee, self.left_ankle, self.right_shoulder,
                         self.right_elbow, self.right_wrist, self.right_hip, self.right_knee,
                         self.right_ankle]).flatten()


    @staticmethod
    def keypoint_distance(kp1_in_ext, kp2_in_ext):
        diff_in_ext = Keypoints()

        diff_in_ext.head = kp1_in_ext.head - kp2_in_ext.head
        diff_in_ext.left_shoulder = kp1_in_ext.left_shoulder - kp2_in_ext.left_shoulder
        diff_in_ext.left_elbow = kp1_in_ext.left_elbow - kp2_in_ext.left_elbow
        diff_in_ext.left_wrist = kp1_in_ext.left_wrist - kp2_in_ext.left_wrist
        diff_in_ext.left_hip = kp1_in_ext.left_hip - kp2_in_ext.left_hip
        diff_in_ext.left_knee = kp1_in_ext.left_knee - kp2_in_ext.left_knee
        diff_in_ext.left_ankle = kp1_in_ext.left_ankle - kp2_in_ext.left_ankle
        diff_in_ext.right_shoulder = kp1_in_ext.right_shoulder - kp2_in_ext.right_shoulder
        diff_in_ext.right_elbow = kp1_in_ext.right_elbow - kp2_in_ext.right_elbow
        diff_in_ext.right_wrist = kp1_in_ext.right_wrist - kp2_in_ext.right_wrist
        diff_in_ext.right_hip = kp1_in_ext.right_hip - kp2_in_ext.right_hip
        diff_in_ext.right_knee = kp1_in_ext.right_knee - kp2_in_ext.right_knee
        diff_in_ext.right_ankle = kp1_in_ext.right_ankle - kp2_in_ext.right_ankle

        distance = 0.0
        distance += np.linalg.norm(diff_in_ext.head)
        distance += np.linalg.norm(diff_in_ext.left_shoulder)
        distance += np.linalg.norm(diff_in_ext.left_elbow)
        distance += np.linalg.norm(diff_in_ext.left_wrist)
        distance += np.linalg.norm(diff_in_ext.left_hip)
        distance += np.linalg.norm(diff_in_ext.left_knee)
        distance += np.linalg.norm(diff_in_ext.left_ankle)
        distance += np.linalg.norm(diff_in_ext.right_shoulder)
        distance += np.linalg.norm(diff_in_ext.right_elbow)
        distance += np.linalg.norm(diff_in_ext.right_wrist)
        distance += np.linalg.norm(diff_in_ext.right_hip)
        distance += np.linalg.norm(diff_in_ext.right_knee)
        distance += np.linalg.norm(diff_in_ext.right_ankle)

        return distance, diff_in_ext
    

    def __str__(self):
        return (f"head            = {self.head.T}\n"
                f"left_shoulder   = {self.left_shoulder.T}\n"
                f"left_elbow      = {self.left_elbow.T}\n"
                f"left_wrist      = {self.left_wrist.T}\n"
                f"left_hip        = {self.left_hip.T}\n"
                f"left_knee       = {self.left_knee.T}\n"
                f"left_ankle      = {self.left_ankle.T}\n"
                f"right_shoulder  = {self.right_shoulder.T}\n"
                f"right_elbow     = {self.right_elbow.T}\n"
                f"right_wrist     = {self.right_wrist.T}\n"
                f"right_hip       = {self.right_hip.T}\n"
                f"right_knee      = {self.right_knee.T}\n"
                f"right_ankle     = {self.right_ankle.T}\n")


class HumanProcess:
    def __init__(self, n_dof=28, n_params=8, n_keypoints=13, sampling_time=0.01):
        self.dt = sampling_time
        self.n_dof = n_dof
        self.n_params = n_params
        self.n_states = 2*n_dof + n_params  # [q, qdot, params]
        self.n_outputs = 3*n_keypoints      # [x, y, z] for each keypoint

        self.x = np.zeros(self.n_states)
        self.q_idx = np.arange(0, n_dof)
        self.qdot_idx = np.arange(n_dof, 2*n_dof)
        self.param_idx = np.arange(2*n_dof, 2*n_dof + n_params)

        # Define the state-space model
        Aq = np.eye(n_dof)
        Aq_qdot = np.eye(n_dof) * self.dt
        Aq_param = np.zeros((n_dof, n_params))

        Aqdot = np.eye(n_dof)
        Aqdot_q = np.zeros((n_dof, n_dof))
        Aqdot_param = np.zeros((n_dof, n_params))

        Aparam = np.eye(n_params)
        Aparam_q = np.zeros((n_params, n_dof))
        Aparam_qdot = np.zeros((n_params, n_dof))
        
        self.A = np.block([[Aq, Aq_qdot, Aq_param],
                           [Aqdot_q, Aqdot, Aqdot_param],
                           [Aparam_q, Aparam_qdot, Aparam]])

        Bq = np.zeros((n_dof, n_dof))
        Bqdot = np.eye(n_dof) * self.dt
        Bparam = np.zeros((n_params, n_dof))

        self.B = np.block([[Bq],
                           [Bqdot],
                           [Bparam]])


    def update_state(self, u):
        self.x = self.A @ self.x + self.B @ u


    def output(self):
        return self.forward_kinematics(self.x[self.q_idx], self.x[self.param_idx])


    def initialize_state(self, x):
        self.x = x


    def trunk_fk(self, q, param):
        shoulder_rotx = q[7]
        hip_rotz = q[8]
        hip_rotx = q[9]
        shoulder_distance = param[0]
        chest_hip_distance = param[1]
        hip_distance = param[2]

        # Transformation matrix from external frame to chest frame
        chest_q = R.from_quat(q[3:7])
        T_ext_chest = np.eye(4)
        T_ext_chest[:3, :3] = chest_q.as_matrix()
        T_ext_chest[:3, 3] = q[:3]

        # Transformation matrix from chest frame to shoulder frame
        T_chest_shoulder = np.eye(4)
        T_chest_shoulder[:3, :3] = R.from_euler('x', shoulder_rotx).as_matrix()
        
        # Transformation matrix from shoulder frame to right shoulder frame 0
        T_shoulder_rshoulder0 = np.eye(4)
        T_shoulder_rshoulder0[:3, :3] = R.from_euler('x', -np.pi * 0.5).as_matrix()

        # Transformation matrix from shoulder0 frame to right shoulder frame
        T_shoulder0_rshoulder = np.eye(4)
        T_shoulder0_rshoulder[:3, 3] = -np.array([0, 0, 0.5 * shoulder_distance])
        T_shoulder_rshoulder = T_shoulder_rshoulder0 @ T_shoulder0_rshoulder

        # Transformation matrix from shoulder frame to left shoulder frame 0
        T_shoulder_lshoulder0 = np.eye(4)
        T_shoulder_lshoulder0[:3, :3] = R.from_euler('x', -np.pi * 0.5).as_matrix()

        # Transformation matrix from lshoulder0 frame to left shoulder frame
        T_lshoulder0_lshoulder = np.eye(4)
        T_lshoulder0_lshoulder[:3, 3] = np.array([0, 0, 0.5 * shoulder_distance])
        T_shoulder_lshoulder = T_shoulder_lshoulder0 @ T_lshoulder0_lshoulder

        # Transformation matrix from chest frame to lshoulder and rshoulder frame
        T_ext_lshoulder = T_ext_chest @ T_chest_shoulder @ T_shoulder_lshoulder
        T_ext_rshoulder = T_ext_chest @ T_chest_shoulder @ T_shoulder_rshoulder

        # Transformation matrix from chest frame to hip frame 0
        T_chest_hip0 = np.eye(4)
        T_chest_hip0[2, 3] = -chest_hip_distance

        T_hip0_hip1 = np.eye(4)
        T_hip0_hip1[:3, :3] = R.from_euler('z', hip_rotz).as_matrix()

        T_hip1_hip2 = np.eye(4)
        T_hip1_hip2[:3, :3] = R.from_euler('x', hip_rotx).as_matrix()

        T_hip2_rhip3 = np.eye(4)
        T_hip2_rhip3[:3, :3] = R.from_euler('x', -np.pi * 0.5).as_matrix()

        T_rhip3_rhip = np.eye(4)
        T_rhip3_rhip[2, 3] = -0.5 * hip_distance

        T_hip2_lhip3 = np.eye(4)
        T_hip2_lhip3[:3, :3] = R.from_euler('x', -np.pi * 0.5).as_matrix()

        T_lhip3_lhip = np.eye(4)
        T_lhip3_lhip[2, 3] = 0.5 * hip_distance

        T_ext_rhip = T_ext_chest @ T_chest_hip0 @ T_hip0_hip1 @ T_hip1_hip2 @ T_hip2_rhip3 @ T_rhip3_rhip
        T_ext_lhip = T_ext_chest @ T_chest_hip0 @ T_hip0_hip1 @ T_hip1_hip2 @ T_hip2_lhip3 @ T_lhip3_lhip

        return T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest


    def head_fk(self, q, param, T_ext_chest):
        q1 = q[0]
        q2 = q[1]

        distance = param

        # Transformation matrix from chest frame to head0 frame
        T_chest_head0 = np.eye(4)
        T_chest_head0[:3, :3] = R.from_euler('x', q1).as_matrix()

        # Transformation matrix from head0 frame to head1 frame
        T_head0_head1 = np.eye(4)
        T_head0_head1[:3, :3] = R.from_euler('y', q2).as_matrix()

        # Transformation matrix from head1 frame to head frame
        T_head1_head = np.eye(4)
        T_head1_head[2, 3] = distance

        T_ext_head = T_ext_chest @ T_chest_head0 @ T_head0_head1 @ T_head1_head
        
        # Check if T_ext_head is a rotation matrix
        assert np.isclose(np.linalg.det(T_ext_head), 1.0), "T_ext_head is not a rotation matrix"

        return T_ext_head 


    def right_limb_fk(self, qarm, param):
        q1 = qarm[0]   # shoulder rot z
        q2 = qarm[1]   # shoulder rot x
        q3 = qarm[2]   # shoulder rot y
        q5 = qarm[3]   # elbow rot z
        q4 = param[0]  # upper arm length
        q6 = param[1]  # lower arm length

        rot01 = R.from_euler('z', q1).as_matrix()
        T01 = np.eye(4)
        T01[:3, :3] = rot01

        rot12 = R.from_euler('x', q2).as_matrix()
        T12 = np.eye(4)
        T12[:3, :3] = rot12

        rot23 = R.from_euler('y', q3).as_matrix()
        T23 = np.eye(4)
        T23[:3, :3] = rot23

        T34 = np.eye(4)
        T34[1, 3] = q4 # translation along y

        rot45 = R.from_euler('z', q5).as_matrix()
        T45 = np.eye(4)
        T45[:3, :3] = rot45

        T56 = np.eye(4)
        T56[1, 3] = q6 # translation along y

        T04 = T01 @ T12 @ T23 @ T34
        T06 = T04 @ T45 @ T56

        elbow_in_limb = T04[:3, 3]
        wrist_in_limb = T06[:3, 3]

        return elbow_in_limb, wrist_in_limb


    def left_limb_fk(self, qarm, param):
        elbow_in_limb, wrist_in_limb = self.right_limb_fk(qarm, param)
        elbow_in_limb[2] *= -1.0
        wrist_in_limb[2] *= -1.0
        return elbow_in_limb, wrist_in_limb
    

    def forward_kinematics(self, q, param):
        # Configuration
        q_trunk = q[0:10]
        q_right_arm = q[10:14]
        q_left_arm = q[14:18]
        q_right_leg = q[18:22]
        q_left_leg = q[22:26]
        q_head = q[26:28]

        # Body parameters
        trunk_param = param[0:3]
        arm_param = param[3:5]
        leg_param = param[5:7]
        head_param = param[7]

        # trunk forward kinematics
        T_ext_rshoulder, T_ext_lshoulder, \
            T_ext_rhip, T_ext_lhip, \
                T_ext_chest = self.trunk_fk(q_trunk, trunk_param)

        # Head forward kinematics
        T_ext_head = self.head_fk(q_head, head_param, T_ext_chest)

        # Extract head and trunk keypoints
        keypoints = {}
        keypoints["head"] = T_ext_head[:3, 3]
        keypoints["right_shoulder"] = T_ext_rshoulder[:3, 3]
        keypoints["left_shoulder"] = T_ext_lshoulder[:3, 3]
        keypoints["right_hip"] = T_ext_rhip[:3, 3]
        keypoints["left_hip"] = T_ext_lhip[:3, 3]

        # Right arm forward kinematics
        relbow_in_rshoulder, rwrist_in_rshoulder = self.right_limb_fk(q_right_arm, arm_param)

        # Left arm forward kinematics
        lelbow_in_lshoulder, lwrist_in_lshoulder = self.left_limb_fk(q_left_arm, arm_param)

        # Right leg forward kinematics
        relbow_in_rhip, rwrist_in_rhip = self.right_limb_fk(q_right_leg, leg_param)

        # Left leg forward kinematics
        lelbow_in_lhip, lwrist_in_lhip = self.left_limb_fk(q_left_leg, leg_param)

        # Convert to homogeneous coordinates
        relbow_in_rshoulder = np.append(relbow_in_rshoulder, 1)
        rwrist_in_rshoulder = np.append(rwrist_in_rshoulder, 1)
        lelbow_in_lshoulder = np.append(lelbow_in_lshoulder, 1)
        lwrist_in_lshoulder = np.append(lwrist_in_lshoulder, 1)
        relbow_in_rhip = np.append(relbow_in_rhip, 1)
        rwrist_in_rhip = np.append(rwrist_in_rhip, 1)
        lelbow_in_lhip = np.append(lelbow_in_lhip, 1)
        lwrist_in_lhip = np.append(lwrist_in_lhip, 1)

        # Extract arm and leg keypoints
        keypoints["right_elbow"] = (T_ext_rshoulder @ relbow_in_rshoulder)[:3]
        keypoints["right_wrist"] = (T_ext_rshoulder @ rwrist_in_rshoulder)[:3]
        keypoints["left_elbow"]  = (T_ext_lshoulder @ lelbow_in_lshoulder)[:3]
        keypoints["left_wrist"]  = (T_ext_lshoulder @ lwrist_in_lshoulder)[:3]
        keypoints["right_knee"]  = (T_ext_rhip @ relbow_in_rhip)[:3]
        keypoints["right_ankle"] = (T_ext_rhip @ rwrist_in_rhip)[:3]
        keypoints["left_knee"]   = (T_ext_lhip @ lelbow_in_lhip)[:3]
        keypoints["left_ankle"]  = (T_ext_lhip @ lwrist_in_lhip)[:3]

        return keypoints


    def right_limb_ik(self, elbow_in_limb, wrist_in_limb, param):
        qarm = np.zeros(4)
        
        q1 = np.arctan2(-elbow_in_limb[0], elbow_in_limb[1])
        if np.abs(np.sin(q1)) > 0.5:
            q2 = np.arctan2(elbow_in_limb[2], -elbow_in_limb[0] / np.sin(q1))
        else:
            q2 = np.arctan2(elbow_in_limb[2], elbow_in_limb[1] / np.cos(q1))
        
        rot01 = R.from_rotvec(q1 * np.array([0, 0, 1]))
        rot12 = R.from_rotvec(q2 * np.array([1, 0, 0]))
        
        T01 = rot01.as_matrix()
        T12 = rot12.as_matrix()
        
        T02 = T01 @ T12
        
        wrist_in_2 = np.linalg.inv(T02) @ wrist_in_limb
        q3 = np.arctan2(wrist_in_2[2], -wrist_in_2[0])
        
        if np.abs(np.sin(q3)) > 0.5:
            q6sinq5 = wrist_in_2[2] / np.sin(q3)
        else:
            q6sinq5 = -wrist_in_2[0] / np.cos(q3)
        
        q6cosq5 = wrist_in_2[1] - param[0]
        
        q5 = np.arctan2(q6sinq5, q6cosq5)
        
        qarm[0] = q1
        qarm[1] = q2
        qarm[2] = q3
        qarm[3] = q5
        
        return qarm


    def left_limb_ik(self, elbow_in_limb, wrist_in_limb, param):
        mirror_elbow_in_limb = elbow_in_limb.copy()
        mirror_wrist_in_limb = wrist_in_limb.copy()
        mirror_elbow_in_limb[2] *= -1.0
        mirror_wrist_in_limb[2] *= -1.0
        return self.right_limb_ik(mirror_elbow_in_limb, mirror_wrist_in_limb, param) 


    def trunk_ik(self, measures_in_ext: Keypoints):
        q = np.zeros(10)
        param = np.zeros(3)

        upper_chest = 0.5 * (measures_in_ext.left_shoulder + measures_in_ext.right_shoulder)
        lower_chest = 0.5 * (measures_in_ext.left_hip + measures_in_ext.right_hip)
        hip_versor_in_ext = (measures_in_ext.left_hip - measures_in_ext.right_hip) / \
            np.linalg.norm(measures_in_ext.left_hip - measures_in_ext.right_hip)
        chest_z_in_ext = (upper_chest - lower_chest) / np.linalg.norm(upper_chest - lower_chest)  
        shoulder_versor_in_ext = (measures_in_ext.left_shoulder - measures_in_ext.right_shoulder) / \
            np.linalg.norm(measures_in_ext.left_shoulder - measures_in_ext.right_shoulder)

        shoulder_distance = np.linalg.norm(measures_in_ext.left_shoulder - measures_in_ext.right_shoulder)
        chest_hip_distance = np.linalg.norm(upper_chest - lower_chest)
        hip_distance = np.linalg.norm(measures_in_ext.left_hip - measures_in_ext.right_hip)

        chest_y_in_ext = shoulder_versor_in_ext - np.dot(shoulder_versor_in_ext, chest_z_in_ext) * chest_z_in_ext
        chest_y_in_ext /= np.linalg.norm(chest_y_in_ext)

        chest_x_in_ext = np.cross(chest_y_in_ext, chest_z_in_ext)

        chest_rot = np.column_stack((chest_x_in_ext, chest_y_in_ext, chest_z_in_ext))
        chest_q = R.from_matrix(chest_rot).as_quat() # type: ignore

        T_ext_chest = np.eye(4)
        T_ext_chest[:3, :3] = chest_rot
        T_ext_chest[:3, 3] = upper_chest

        q[:3] = upper_chest
        q[3:7] = chest_q

        shoulder_versor_in_chest = np.linalg.inv(T_ext_chest[:3, :3]) @ shoulder_versor_in_ext
        shoulder_rotx = np.arctan2(shoulder_versor_in_chest[2], shoulder_versor_in_chest[1])

        hip_versor_in_chest = np.linalg.inv(T_ext_chest[:3, :3]) @ hip_versor_in_ext
        hip_rotz = np.arctan2(-hip_versor_in_chest[0], hip_versor_in_chest[1])

        if np.abs(np.sin(hip_rotz)) > 0.5:
            cosq2 = -hip_versor_in_chest[0] / np.sin(hip_rotz)
        else:
            cosq2 = hip_versor_in_chest[1] / np.cos(hip_rotz)

        hip_rotx = np.arctan2(hip_versor_in_chest[2], cosq2)

        q[7] = shoulder_rotx
        q[8] = hip_rotz
        q[9] = hip_rotx

        param[0] = shoulder_distance
        param[1] = chest_hip_distance
        param[2] = hip_distance

        return q, param
    

    def head_ik(self, measures_in_ext: Keypoints, T_ext_chest):
        param = np.zeros(1)
        q = np.zeros(2)

        head_in_ext = np.concatenate([measures_in_ext.head, np.array([1])])
        head_in_chest = np.linalg.inv(T_ext_chest) @ head_in_ext
        param[0] = np.linalg.norm(head_in_chest)

        q1 = np.arctan2(-head_in_chest[1], head_in_chest[2])
        if np.abs(np.sin(q1)) > 0.5:
            dcosq2 = -head_in_chest[1] / np.sin(q1)
        else:
            dcosq2 = head_in_chest[2] / np.cos(q1)

        q2 = np.arctan2(head_in_chest[0], dcosq2)

        q[0] = q1
        q[1] = q2

        return q, param


    def inverse_kinematics(self, measures_in_ext: Keypoints):
        # 7 dof for chest (tra+quat)
        # 1 dof: shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
        # 1 dof for trunk rotation (around chest_z)
        # 1 dof: hip rotation is the rotation around chest_x_in_ext (frontal direction)
        # 3 dof translation= shoulder_distance, chest_hip_distance, hip_distance
        # 6 dof for each limb: 3 dof shoulder, 1 dof length of the upper arm, 1 dof elbow rotation, 1 dof length of the lower arm
        q_trunk, trunk_param = self.trunk_ik(measures_in_ext)

        T_ext_rshoulder, T_ext_lshoulder, \
        T_ext_rhip, T_ext_lhip, \
        T_ext_chest = \
            self.trunk_fk(q_trunk, trunk_param)

        q_head, head_param = self.head_ik(measures_in_ext, T_ext_chest)

        upper_arm_length = 0.5 * (
            np.linalg.norm(measures_in_ext.right_elbow - measures_in_ext.right_shoulder) +
            np.linalg.norm(measures_in_ext.left_elbow - measures_in_ext.left_shoulder)
        )

        lower_arm_length = 0.5 * (
            np.linalg.norm(measures_in_ext.right_elbow - measures_in_ext.right_wrist) +
            np.linalg.norm(measures_in_ext.left_elbow - measures_in_ext.left_wrist)
        )

        upper_leg_length = 0.5 * (
            np.linalg.norm(measures_in_ext.right_knee - measures_in_ext.right_hip) +
            np.linalg.norm(measures_in_ext.left_knee - measures_in_ext.left_hip)
        )

        lower_leg_length = 0.5 * (
            np.linalg.norm(measures_in_ext.right_knee - measures_in_ext.right_ankle) +
            np.linalg.norm(measures_in_ext.left_knee - measures_in_ext.left_ankle)
        )

        arm_param = np.array([upper_arm_length, lower_arm_length])
        leg_param = np.array([upper_leg_length, lower_leg_length])

        relbow_in_ext = np.concatenate([measures_in_ext.right_elbow, np.array([1])])
        relbow_in_rshoulder = np.linalg.inv(T_ext_rshoulder) @ relbow_in_ext

        rwrist_in_ext = np.concatenate([measures_in_ext.right_wrist, np.array([1])])
        rwrist_in_rshoulder = np.linalg.inv(T_ext_rshoulder) @ rwrist_in_ext

        lelbow_in_ext = np.concatenate([measures_in_ext.left_elbow, np.array([1])])
        lelbow_in_lshoulder = np.linalg.inv(T_ext_lshoulder) @ lelbow_in_ext

        lwrist_in_ext = np.concatenate([measures_in_ext.left_wrist, np.array([1])])
        lwrist_in_lshoulder = np.linalg.inv(T_ext_lshoulder) @ lwrist_in_ext

        q_right_arm = self.right_limb_ik(relbow_in_rshoulder[:-1], rwrist_in_rshoulder[:-1], arm_param)
        q_left_arm = self.left_limb_ik(lelbow_in_lshoulder[:-1], lwrist_in_lshoulder[:-1], arm_param)

        rknee_in_ext = np.concatenate([measures_in_ext.right_knee, np.array([1])])
        rknee_in_rhip = np.linalg.inv(T_ext_rhip) @ rknee_in_ext

        rankle_in_ext = np.concatenate([measures_in_ext.right_ankle, np.array([1])])
        rankle_in_rhip = np.linalg.inv(T_ext_rhip) @ rankle_in_ext

        lknee_in_ext = np.concatenate([measures_in_ext.left_knee, np.array([1])])
        lknee_in_lhip = np.linalg.inv(T_ext_lhip) @ lknee_in_ext

        lankle_in_ext = np.concatenate([measures_in_ext.left_ankle, np.array([1])])
        lankle_in_lhip = np.linalg.inv(T_ext_lhip) @ lankle_in_ext

        q_right_leg = self.right_limb_ik(rknee_in_rhip[:-1], rankle_in_rhip[:-1], leg_param)
        q_left_leg = self.left_limb_ik(lknee_in_lhip[:-1], lankle_in_lhip[:-1], leg_param)

        configuration = np.concatenate([q_trunk, q_right_arm, q_left_arm,
                                        q_right_leg, q_left_leg, q_head])
        param = np.concatenate([trunk_param, arm_param, leg_param, head_param])

        return configuration, param
    

    def print(self, q, param):
        q_trunk = q[0:10]
        q_right_arm = q[10:14]
        q_left_arm = q[14:18]
        q_right_leg = q[18:22]
        q_left_leg = q[22:26]
        q_head = q[26:28]

        print(f"chest position. x: {q_trunk[0]}, y: {q_trunk[1]}, z: {q_trunk[2]}")
        print(f"chest quaternion. x: {q_trunk[3]}, y: {q_trunk[4]}, z: {q_trunk[5]}, w: {q_trunk[6]}\n")
        print(f"shoulder rotx: {q_trunk[7]}")
        print(f"hip rotz: {q_trunk[8]}")
        print(f"hip rotx: {q_trunk[9]}\n")

        qlimb = q_right_arm
        print("right arm:")
        print(f"1) rotz: {qlimb[0]}")
        print(f"2) rotx: {qlimb[1]}")
        print(f"3) roty: {qlimb[2]}")
        print(f"4) rotz: {qlimb[3]}\n")

        qlimb = q_left_arm
        print("left arm:")
        print(f"1) rotz: {qlimb[0]}")
        print(f"2) rotx: {qlimb[1]}")
        print(f"3) roty: {qlimb[2]}")
        print(f"4) rotz: {qlimb[3]}\n")

        qlimb = q_right_leg
        print("right leg:")
        print(f"1) rotz: {qlimb[0]}")
        print(f"2) rotx: {qlimb[1]}")
        print(f"3) roty: {qlimb[2]}")
        print(f"4) rotz: {qlimb[3]}\n")

        qlimb = q_left_leg
        print("left leg:")
        print(f"1) rotz: {qlimb[0]}")
        print(f"2) rotx: {qlimb[1]}")
        print(f"3) roty: {qlimb[2]}")
        print(f"4) rotz: {qlimb[3]}\n")

        print(f"head rotz: {q_head[0]}")
        print(f"head rotx: {q_head[1]}\n")