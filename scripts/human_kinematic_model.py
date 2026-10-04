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


class JointLimits:
    def __init__(self, min, max):
        self.min = min
        self.max = max


# Quaternion helpers with the same conventions and algorithms as Eigen: quaternions are (x, y, z, w)


def quat_to_rotmat(quat):
    """Rotation matrix of a quaternion (x, y, z, w), as Eigen::Quaterniond::toRotationMatrix().

    The quaternion is not normalized here: use normalize_quat first for non-unit quaternions.
    """
    x, y, z, w = quat
    tx, ty, tz = 2 * x, 2 * y, 2 * z
    twx, twy, twz = tx * w, ty * w, tz * w
    txx, txy, txz = tx * x, ty * x, tz * x
    tyy, tyz, tzz = ty * y, tz * y, tz * z
    return np.array([[1 - (tyy + tzz), txy - twz, txz + twy],
                     [txy + twz, 1 - (txx + tzz), tyz - twx],
                     [txz - twy, tyz + twx, 1 - (txx + tyy)]])


def normalize_quat(quat):
    """Normalized quaternion, as Eigen::Quaterniond::normalize(): a zero quaternion is left unchanged."""
    quat = np.asarray(quat, dtype=float)
    squared_norm = np.dot(quat, quat)
    return quat / np.sqrt(squared_norm) if squared_norm > 0 else quat


def rotmat_to_quat(mat):
    """Quaternion (x, y, z, w) of a rotation matrix, as Eigen::Quaterniond(Matrix3d) (Shoemake's algorithm)."""
    quat = np.zeros(4)
    t = np.trace(mat)
    if t > 0:
        t = np.sqrt(t + 1.0)
        quat[3] = 0.5 * t
        t = 0.5 / t
        quat[0] = (mat[2, 1] - mat[1, 2]) * t
        quat[1] = (mat[0, 2] - mat[2, 0]) * t
        quat[2] = (mat[1, 0] - mat[0, 1]) * t
    else:
        i = 0
        if mat[1, 1] > mat[0, 0]:
            i = 1
        if mat[2, 2] > mat[i, i]:
            i = 2
        j = (i + 1) % 3
        k = (j + 1) % 3
        t = np.sqrt(mat[i, i] - mat[j, j] - mat[k, k] + 1.0)
        quat[i] = 0.5 * t
        t = 0.5 / t
        quat[3] = (mat[k, j] - mat[j, k]) * t
        quat[j] = (mat[j, i] + mat[i, j]) * t
        quat[k] = (mat[k, i] + mat[i, k]) * t
    return quat


def quat_multiply(a, b):
    """Hamilton product a * b of quaternions (x, y, z, w)."""
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return np.array([aw * bx + ax * bw + ay * bz - az * by,
                     aw * by + ay * bw + az * bx - ax * bz,
                     aw * bz + az * bw + ax * by - ay * bx,
                     aw * bw - ax * bx - ay * by - az * bz])


def canonical_quat(quat):
    """Quaternion with a non-negative scalar part (consistent representation used by the model)."""
    return -quat if quat[3] < 0 else quat


def chest_quat_rotated(chest_q):
    """Chest quaternion rotated by 180 deg about its z axis (Human28DOF::chestQuatRotated)."""
    half_angle = 0.5 * np.pi
    return canonical_quat(quat_multiply(chest_q, np.array([0.0, 0.0, np.sin(half_angle), np.cos(half_angle)])))



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
        # (the quaternion is normalized as in the C++ code; a zero quaternion is left unchanged)
        T_ext_chest = np.eye(4)
        T_ext_chest[:3, :3] = quat_to_rotmat(normalize_quat(q[3:7]))
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


    # =======================================================================
    # Inverse kinematics: same algorithm as Human28DOF (src/human_model/human_model.cpp)
    # =======================================================================

    @staticmethod
    def _in_bounds(x, limits: JointLimits):
        # strict bounds, False for NaN (as in the C++ code)
        return limits.min < x < limits.max


    @staticmethod
    def _div_sin_or_cos(num_if_sin, num_if_cos, angle):
        # num_if_sin / sin(angle) if |sin(angle)| > 0.5, else num_if_cos / cos(angle)
        if np.abs(np.sin(angle)) > 0.5:
            return num_if_sin / np.sin(angle)
        return num_if_cos / np.cos(angle)


    def _shoulder_ik(self, elbow_in_limb, qshoulder_bounds: list[JointLimits], first_solution: bool):
        """Shoulder rot z and rot x (Human28DOF::shoulderIk). Returns (q (2,), valid), q is NaN if not valid."""
        e = elbow_in_limb
        if first_solution:
            # Solution 1: hypothesis -pi/2 < q2 < pi/2 (cos(q2) > 0)
            q1 = np.arctan2(-e[0], e[1])
        else:
            # Solution 2: hypothesis -pi < q2 < -pi/2 or pi/2 < q2 < pi (cos(q2) < 0)
            q1 = np.arctan2(e[0], -e[1])
        q2 = np.arctan2(e[2], self._div_sin_or_cos(-e[0], e[1], q1))

        valid = self._in_bounds(q1, qshoulder_bounds[0]) and self._in_bounds(q2, qshoulder_bounds[1])
        valid = valid and ((np.cos(q2) > 0) if first_solution else (np.cos(q2) < 0))

        q = np.array([q1, q2]) if valid else np.full(2, np.nan)
        return q, valid


    def _elbow_ik(self, wrist_in_2, upper_arm_length, qelbow_bounds: list[JointLimits], first_solution: bool):
        """Shoulder rot y and elbow rot z (Human28DOF::elbowIk). Returns (q (2,), valid), q is NaN if not valid."""
        w = wrist_in_2
        q6cosq5 = w[1] - upper_arm_length
        if first_solution:
            # Solution 1: hypothesis 0 < q5 < pi (sin(q5) > 0)
            q3 = np.arctan2(w[2], -w[0])
        else:
            # Solution 2: hypothesis -pi < q5 < 0 (sin(q5) < 0)
            q3 = np.arctan2(-w[2], w[0])
        q5 = np.arctan2(self._div_sin_or_cos(w[2], -w[0], q3), q6cosq5)

        valid = self._in_bounds(q3, qelbow_bounds[0]) and self._in_bounds(q5, qelbow_bounds[1])
        valid = valid and ((np.sin(q5) > 0) if first_solution else (np.sin(q5) < 0))

        q = np.array([q3, q5]) if valid else np.full(2, np.nan)
        return q, valid


    @staticmethod
    def _wrist_in_2(qshoulder, wrist_in_limb):
        """Wrist position in the frame after shoulder rot z and rot x (Human28DOF::computeWristIn2)."""
        T02 = R.from_euler('z', qshoulder[0]).as_matrix() @ R.from_euler('x', qshoulder[1]).as_matrix()
        return np.linalg.inv(T02) @ wrist_in_limb


    def right_limb_ik(self, elbow_in_limb, wrist_in_limb, param, qarm_bounds: list[JointLimits], qarm_previous):
        """Closed-form limb IK (Human28DOF::rightLimbIk).

        Up to four solutions are computed (2 for the shoulder x 2 for the elbow); among the ones
        within the joint limits, the closest to qarm_previous is returned (NaN if none is valid).
        qarm_bounds are the limits of (shoulder rot z, shoulder rot x, shoulder rot y, elbow rot z).
        """
        q_shoulder_bounds = qarm_bounds[0:2]
        q_elbow_bounds = qarm_bounds[2:4]

        q_distance = np.inf
        qarm = np.full(4, np.nan)
        for first_shoulder in (True, False):
            q_shoulder, valid_shoulder = self._shoulder_ik(elbow_in_limb, q_shoulder_bounds, first_shoulder)
            wrist_in_2 = self._wrist_in_2(q_shoulder, wrist_in_limb)
            for first_elbow in (True, False):
                q_elbow, valid_elbow = self._elbow_ik(wrist_in_2, param[0], q_elbow_bounds, first_elbow)
                if valid_shoulder and valid_elbow:
                    # Human28DOF::updateIfCloser: replace only if strictly closer
                    qarm_temp = np.concatenate([q_shoulder, q_elbow])
                    q_distance_temp = np.linalg.norm(qarm_temp - qarm_previous)
                    if q_distance_temp < q_distance:
                        qarm = qarm_temp
                        q_distance = q_distance_temp

        return qarm


    def left_limb_ik(self, elbow_in_limb, wrist_in_limb, param, qarm_bounds: list[JointLimits], qarm_previous):
        mirror_elbow_in_limb = np.array(elbow_in_limb, dtype=float)
        mirror_wrist_in_limb = np.array(wrist_in_limb, dtype=float)
        mirror_elbow_in_limb[2] *= -1.0
        mirror_wrist_in_limb[2] *= -1.0
        return self.right_limb_ik(mirror_elbow_in_limb, mirror_wrist_in_limb, param, qarm_bounds, qarm_previous)


    def trunk_ik(self, measures_in_ext: Keypoints, qtrunk_bounds: list[JointLimits]):
        """Trunk IK (Human28DOF::trunkIk).

        qtrunk_bounds are the limits of (shoulder rot x, hip rot z, hip rot x).
        Returns q_trunk (10,), param_trunk (3,), chest_q_rotated (4,).
        If the shoulder rotation is out of bounds (the bounds themselves are allowed), it is NaN
        (and so are both arms in inverse_kinematics), as for the other invalid solutions.
        """
        q = np.zeros(10)
        param = np.zeros(3)

        # Compute the versors of the shoulders and the hips
        shoulder_versor_in_ext = (measures_in_ext.left_shoulder - measures_in_ext.right_shoulder) / \
            np.linalg.norm(measures_in_ext.left_shoulder - measures_in_ext.right_shoulder)
        hip_versor_in_ext = (measures_in_ext.left_hip - measures_in_ext.right_hip) / \
            np.linalg.norm(measures_in_ext.left_hip - measures_in_ext.right_hip)

        # Compute the chest reference frame Z
        upper_chest = 0.5 * (measures_in_ext.left_shoulder + measures_in_ext.right_shoulder)
        lower_chest = 0.5 * (measures_in_ext.left_hip + measures_in_ext.right_hip)
        chest_z_in_ext = (upper_chest - lower_chest) / np.linalg.norm(upper_chest - lower_chest)

        # Compute the distances between the shoulders and the hips
        param[0] = np.linalg.norm(measures_in_ext.left_shoulder - measures_in_ext.right_shoulder)  # shoulder distance
        param[1] = np.linalg.norm(upper_chest - lower_chest)                                     # chest-hip distance
        param[2] = np.linalg.norm(measures_in_ext.left_hip - measures_in_ext.right_hip)          # hip distance

        # Chest reference frame: x frontal, y right to left shoulder, z lower to upper chest
        chest_y_in_ext = shoulder_versor_in_ext - np.dot(shoulder_versor_in_ext, chest_z_in_ext) * chest_z_in_ext
        chest_y_in_ext /= np.linalg.norm(chest_y_in_ext)
        chest_x_in_ext = np.cross(chest_y_in_ext, chest_z_in_ext)
        chest_rot = np.column_stack((chest_x_in_ext, chest_y_in_ext, chest_z_in_ext))

        # Convert to quaternion, with a non-negative scalar part for a consistent representation
        chest_q = canonical_quat(rotmat_to_quat(chest_rot))
        chest_q_rotated = chest_quat_rotated(chest_q)

        # The chest frame is rebuilt from the quaternion, as in the C++ code
        R_ext_chest = quat_to_rotmat(chest_q)

        q[:3] = upper_chest
        q[3:7] = chest_q

        # Shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
        shoulder_versor_in_chest = np.linalg.inv(R_ext_chest) @ shoulder_versor_in_ext
        shoulder_rotx = np.arctan2(shoulder_versor_in_chest[2], shoulder_versor_in_chest[1])
        if shoulder_rotx < qtrunk_bounds[0].min or shoulder_rotx > qtrunk_bounds[0].max:
            shoulder_rotx = np.nan

        # Hip rot z and hip rot x
        h = np.linalg.inv(R_ext_chest) @ hip_versor_in_ext

        # Solution 1: hypothesis -pi/2 < hip_rotx < pi/2 (cos(hip_rotx) > 0)
        hip_rotz = np.arctan2(-h[0], h[1])
        hip_rotx = np.arctan2(h[2], self._div_sin_or_cos(-h[0], h[1], hip_rotz))
        valid = (np.cos(hip_rotx) > 0
                 and self._in_bounds(hip_rotz, qtrunk_bounds[1]) and self._in_bounds(hip_rotx, qtrunk_bounds[2]))

        if not valid:
            # Solution 2: hypothesis -pi < hip_rotx < -pi/2 or pi/2 < hip_rotx < pi (cos(hip_rotx) < 0)
            hip_rotz = np.arctan2(h[0], -h[1])
            hip_rotx = np.arctan2(h[2], self._div_sin_or_cos(-h[0], h[1], hip_rotz))
            valid = (np.cos(hip_rotx) < 0
                     and self._in_bounds(hip_rotz, qtrunk_bounds[1]) and self._in_bounds(hip_rotx, qtrunk_bounds[2]))

        q[7] = shoulder_rotx
        q[8] = hip_rotz if valid else np.nan
        q[9] = hip_rotx if valid else np.nan

        return q, param, chest_q_rotated


    def head_ik(self, measures_in_ext: Keypoints, T_ext_chest, qhead_bounds: list[JointLimits]):
        """Head IK (Human28DOF::headIk). qhead_bounds are the limits of (head rot x, head rot y).
        Returns q_head (2,), param_head (1,)."""
        # Express head_in_ext in homogeneous coordinates, compute head_in_chest and remove the homogeneous coordinate
        head_in_ext = np.concatenate([measures_in_ext.head, np.array([1])])
        head_in_chest = (np.linalg.inv(T_ext_chest) @ head_in_ext)[:-1]

        param = np.array([np.linalg.norm(head_in_chest)])

        # Solution 1: hypothesis cos(q2) > 0
        q1 = np.arctan2(-head_in_chest[1], head_in_chest[2])
        q2 = np.arctan2(head_in_chest[0], self._div_sin_or_cos(-head_in_chest[1], head_in_chest[2], q1))
        valid = (np.cos(q2) > 0
                 and self._in_bounds(q1, qhead_bounds[0]) and self._in_bounds(q2, qhead_bounds[1]))

        if not valid:
            # Solution 2: hypothesis cos(q2) < 0
            q1 = np.arctan2(head_in_chest[1], -head_in_chest[2])
            q2 = np.arctan2(head_in_chest[0], self._div_sin_or_cos(-head_in_chest[1], head_in_chest[2], q1))
            valid = (np.cos(q2) < 0
                     and self._in_bounds(q1, qhead_bounds[0]) and self._in_bounds(q2, qhead_bounds[1]))

        q = np.array([q1, q2]) if valid else np.full(2, np.nan)
        return q, param


    def inverse_kinematics(self, measures_in_ext: Keypoints, qbounds: list[JointLimits], q_previous):
        """Inverse kinematics (Human28DOF::ik).

        Args:
            measures_in_ext: keypoints in the external frame
            qbounds: 28 joint limits (see Human28DOF::setDefaultJointLimits)
            q_previous: (28,) previous configuration, used to choose among multiple limb solutions

        Returns:
            configuration (28,), param (8,), chest_q_rotated (4,)
        """
        # 7 dof for chest (tra+quat)
        # 1 dof: shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
        # 1 dof for trunk rotation (around chest_z)
        # 1 dof: hip rotation is the rotation around chest_x_in_ext (frontal direction)
        # 3 dof translation= shoulder_distance, chest_hip_distance, hip_distance
        # 6 dof for each limb: 3 dof shoulder, 1 dof length of the upper arm, 1 dof elbow rotation, 1 dof length of the lower arm
        q_previous = np.asarray(q_previous, dtype=float)

        q_trunk, trunk_param, chest_q_rotated = self.trunk_ik(measures_in_ext, qbounds[7:10])

        T_ext_rshoulder, T_ext_lshoulder, \
        T_ext_rhip, T_ext_lhip, \
        T_ext_chest = \
            self.trunk_fk(q_trunk, trunk_param)

        q_head, head_param = self.head_ik(measures_in_ext, T_ext_chest, qbounds[26:28])

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

        def in_frame(T, point):
            return (np.linalg.inv(T) @ np.concatenate([point, np.array([1])]))[:-1]

        q_right_arm = self.right_limb_ik(in_frame(T_ext_rshoulder, measures_in_ext.right_elbow),
                                         in_frame(T_ext_rshoulder, measures_in_ext.right_wrist),
                                         arm_param, qbounds[10:14], q_previous[10:14])
        q_left_arm = self.left_limb_ik(in_frame(T_ext_lshoulder, measures_in_ext.left_elbow),
                                       in_frame(T_ext_lshoulder, measures_in_ext.left_wrist),
                                       arm_param, qbounds[14:18], q_previous[14:18])

        q_right_leg = self.right_limb_ik(in_frame(T_ext_rhip, measures_in_ext.right_knee),
                                         in_frame(T_ext_rhip, measures_in_ext.right_ankle),
                                         leg_param, qbounds[18:22], q_previous[18:22])
        q_left_leg = self.left_limb_ik(in_frame(T_ext_lhip, measures_in_ext.left_knee),
                                       in_frame(T_ext_lhip, measures_in_ext.left_ankle),
                                       leg_param, qbounds[22:26], q_previous[22:26])

        configuration = np.concatenate([q_trunk, q_right_arm, q_left_arm,
                                        q_right_leg, q_left_leg, q_head])
        param = np.concatenate([trunk_param, arm_param, leg_param, head_param])

        return configuration, param, chest_q_rotated


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