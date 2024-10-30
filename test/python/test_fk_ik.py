import numpy as np
from human_kinematic_model import Keypoints, HumanProcess

def test_kinematics():
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

    # DEBUG: set random seed
    np.random.seed(0)

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

    for idx in range(10**4):

        # Randomly generate configuration vector
        q = np.random.rand(n_dof)
        q[3:7] /= np.linalg.norm(q[3:7]) # Normalize chest rotation quaternion

        # Create human kinematic model
        model = HumanProcess(n_dof=n_dof, n_params=n_param)
        
        # === Test inverse kinematics ===
        
        # Forward kinematics
        kpts = model.forward_kinematics(q, param)
        kp_in_ext = Keypoints()
        kp_in_ext.set_keypoints(kpts)

        # Inverse kinematics
        q2, param2 = model.inverse_kinematics(kp_in_ext)

        # Print and assert results
        print("\nq [original]: ", q)
        print("\nq2 [after ik]: ", q2)
        print("\nparam [original]: ", param)
        print("\nparam2 [after ik]: ", param2)

        assert all(np.abs(q-q2) < 1e-8), "q2 computed by IK is not equal to original q"
        assert all(np.abs(param-param2) < 1e-8), "param2 computed by IK is not equal to original param"

        # === Test forward kinematics ===

        kpts2 = model.forward_kinematics(q2, param2)
        kp_in_ext2 = Keypoints()
        kp_in_ext2.set_keypoints(kpts2)

        # Compute distance between keypoints computed by forward kinematics
        distance, diff_in_ext = Keypoints.keypoint_distance(kp_in_ext, kp_in_ext2)

        # Print and assert results
        print("\n\nkp_in_ext [original]: ", kp_in_ext)
        print("\nkp_in_ext2 [after fk]: ", kp_in_ext2)
        print("\ndiff_in_ext: ", diff_in_ext)
        print("\ndistance: ", distance)

        assert distance < 1e-8, "distance between keypoints is greater than threshold after FK"
        assert np.linalg.norm(param-param2) < 1e-8, "param difference is greater than threshold"

        # # Print results
        # print("\n\nq: ", q)
        # print("q2: ", q2)
        # print("diff q: ", q-q2)
        # print("param: ", param)
        # print("param2: ", param2)
        # print("diff param: ", np.linalg.norm(param-param2))
        # print("kp_in_ext: ", kp_in_ext)
        # print("kp_in_ext2: ", kp_in_ext2)
        # print("diff kp_in_ext: ", diff_in_ext)
        # print("distance: ", distance)

        # # Assert results
        # assert distance < 1e-8, "distance is greater than threshold"


def main():
    test_kinematics()


if __name__ == '__main__':
    main()