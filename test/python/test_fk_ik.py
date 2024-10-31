import numpy as np
from human_kinematic_model import Keypoints, HumanProcess

TEST_Q = np.array([ 0.680375, -0.211234,  0.566198,  0.485962,  0.670301,
                   -0.492489, -0.268313,  0.536459, -0.444451,  0.10794,
                   -0.0452059, 0.257742, -0.270431,  0.0268018, 0.904459,
                    0.83239,   0.271423,  0.434594, -0.716795,  0.213938,
                   -0.967399, -0.514226, -0.725537,  0.608354, -0.686642,
                   -0.198111, -0.740419, -0.782382])

TEST_KPTS = {
    "head":           np.array([0.687107, -0.544971  , 0.345802]),
    "left_shoulder":  np.array([0.666024, -0.236367  , 0.419017]),
    "left_elbow":     np.array([0.862801, -0.298988  , 0.636634]),
    "left_wrist":     np.array([0.962695, -0.443399  , 0.879876]),
    "left_hip":       np.array([1.02737 , -0.00312208, 0.599881]),
    "left_knee":      np.array([1.09435 ,  0.168946  , 0.897213]),
    "left_ankle":     np.array([1.09923 ,  0.340047  , 1.25874 ]),
    "right_shoulder": np.array([0.694727, -0.186102  , 0.71338 ]),
    "right_elbow":    np.array([0.948553, -0.0811031 , 0.592767]),
    "right_wrist":    np.array([1.20627 ,  0.0174684 , 0.475016]),
    "right_hip":      np.array([1.00407 , -0.0997844 , 0.829257]),
    "right_knee":     np.array([1.11579 ,  0.225018  , 0.896494]),
    "right_ankle":    np.array([1.12542 ,  0.615157  , 0.80875 ])
}
TEST_KPT = Keypoints()
TEST_KPT.set_keypoints(TEST_KPTS)


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

    for _ in range(10**4):
        # Randomly generate configuration vector
        # q = np.random.rand(n_dof)
        # q[3:7] /= np.linalg.norm(q[3:7]) # Normalize chest rotation quaternion
        q = TEST_Q

        # Create human kinematic model
        model = HumanProcess(n_dof=n_dof, n_params=n_param)
        
        # === Test Forward Kinematics ===
        kpts = model.forward_kinematics(q, param)
        kp_in_ext = Keypoints()
        kp_in_ext.set_keypoints(kpts)

        # Print and assert results
        print("\nkp_in_ext [original]:\n", kp_in_ext)
        print("\nkp_in_ext [test]:\n", TEST_KPT)

        assert np.allclose(kp_in_ext.get_keypoints(), TEST_KPT.get_keypoints(), atol=1.e-4), \
            "kp_in_ext is not equal to TEST_KPT. Error in forward kinematics."
        # ===============================

        # === Test Inverse Kinematics ===
        q2, param2 = model.inverse_kinematics(kp_in_ext)

        # Print and assert results
        print("\nq [original]:\n", q)
        print("\nq2 [after ik]:\n", q2)
        print("\nparam [original]:\n", param)
        print("\nparam2 [after ik]:\n", param2)

        assert all(np.abs(q-q2) < 1e-8), "q2 computed by IK is not equal to original q"
        assert all(np.abs(param-param2) < 1e-8), "param2 computed by IK is not equal to original param"
        # === Test Inverse Kinematics ===






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