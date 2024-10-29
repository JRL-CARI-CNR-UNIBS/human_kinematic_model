import numpy as np
from human_kinematic_model import Keypoints, HumanProcess

def test_kinematic():
    shoulder_distance  = 0.3
    chest_hip_distance = 0.4
    hip_distance       = 0.25

    upper_arm_length = 0.3
    lower_arm_length = 0.3
    upper_leg_length = 0.35
    lower_leg_length = 0.4
    head_distance    = 0.4

    n_dof = 7+3+4*4+2
    n_param = 3+2+2+1

    for _ in range(10**4):
        q = np.random.rand(n_dof)

        # Normalize quaternion corresponding to the rotation of the chest
        q[3:7] /= np.linalg.norm(q[3:7]) 

        param = np.array([
            shoulder_distance,
            chest_hip_distance,
            hip_distance,
            upper_arm_length,
            lower_arm_length,
            upper_leg_length,
            lower_leg_length,
            head_distance
        ])

        kp_in_ext = Keypoints()
        kp_in_ext.left_hip[0] = 1.0

        model = HumanProcess(n_dof=n_dof, n_params=n_param)
        
        kpts = model.forward_kinematics(q, param)
        kp_in_ext = Keypoints()
        kp_in_ext.set_keypoints(kpts)

        q2, param2 = model.inverse_kinematics(kp_in_ext)

        kpts2 = model.forward_kinematics(q2, param2)
        kp_in_ext2 = Keypoints()
        kp_in_ext2.set_keypoints(kpts2)

        distance, diff_in_ext = Keypoints.keypoint_distance(kp_in_ext, kp_in_ext2)

        # Print results
        print("q: ", q)
        print("q2: ", q2)
        print("diff q: ", q-q2)
        print("param: ", param)
        print("param2: ", param2)
        print("diff param: ", np.linalg.norm(param-param2))
        print("kp_in_ext: ", kp_in_ext)
        print("kp_in_ext2: ", kp_in_ext2)
        print("diff kp_in_ext: ", diff_in_ext)
        print("distance: ", distance)

        # Assert results
        assert distance < 1e-8, "distance is greater than threshold"
        assert np.linalg.norm(param-param2) < 1e-8, "param difference is greater than threshold"


def main():
    test_kinematic()


if __name__ == '__main__':
    main()