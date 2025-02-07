import time
import numpy as np
from human_model_binding import Keypoints, JointLimits, Human28DOF

# Define test configuration vector
TEST_Q = np.array([ 0.680375, -0.211234,  0.566198,  0.485962,  0.670301,
                   -0.492489, -0.268313,  0.536459, -0.444451,  0.10794,
                   -0.0452059, 0.257742, -0.270431,  0.0268018, 0.904459,
                    0.83239,   0.271423,  0.434594, -0.716795,  0.213938,
                   -0.967399, -0.514226, -0.725537,  0.608354, -0.686642,
                   -0.198111, -0.740419, -0.782382])

# Define expected test keypoints
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

# Define human parameters
shoulder_distance  = 0.3
chest_hip_distance = 0.4
hip_distance       = 0.25

upper_arm_length = 0.3
lower_arm_length = 0.3
upper_leg_length = 0.35
lower_leg_length = 0.4
head_distance    = 0.4

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

# Define number of degrees of freedom and parameters
n_dof = 7+3+4*4+2
n_param = 3+2+2+1

# Initialize joint limits
qbounds = [JointLimits(-np.pi, np.pi)]*n_dof

# Set the shoulder rot y joint limits
qbounds[12] = JointLimits(-np.pi/2, np.pi/2) # right shoulder
qbounds[16] = JointLimits(-np.pi/2, np.pi/2) # left shoulder
qbounds[20] = JointLimits(-np.pi/2, np.pi/2) # right hip
qbounds[24] = JointLimits(-np.pi/2, np.pi/2) # left hip


def test_fk():
    q = TEST_Q

    # Create human kinematic model
    model = Human28DOF()

    # Create Keypoints object
    kp_in_ext = Keypoints()
    
    # === Test Forward Kinematics ===
    model.forward_kinematics(q, param, kp_in_ext)

    # Print and assert results
    print("\nkp_in_ext [original]:\n", kp_in_ext.to_string())
    print("\nkp_in_ext [test]:\n", TEST_KPT.to_string())

    assert np.allclose(kp_in_ext.get_keypoints(), TEST_KPT.get_keypoints(), atol=1.e-4), \
        "kp_in_ext is not equal to TEST_KPT. Error in forward kinematics."


def test_kinematics():
    start_time = time.time()

    # Test the fk and ik functions
    for idx in range(10**4):
        print("\n===============================================================")
        print("Test iteration: ", idx)

        # Randomly generate configuration vector
        q = np.random.rand(n_dof)
        q[3:7] /= np.linalg.norm(q[3:7]) # Normalize chest rotation quaternion

        # If the scalar part of the quaternion is negative,
        # multiply by -1 to ensure consistent representation
        if q[6] < 0:
            q[3:7] *= -1.0

        print("\nq [original]: \n", q)
        print("\nparam [original]: \n", param)

        # Create human kinematic model
        model = Human28DOF()

        kp_in_ext = Keypoints()
        model.forward_kinematics(q, param, kp_in_ext)

        q2 = np.zeros(n_dof)
        param2 = np.zeros(n_param)
        q2, param2 = model.inverse_kinematics(kp_in_ext, qbounds, q, q2, param2) # use q as previous configuration

        print("\nq2 [before fk]:        \n", q2)
        print("\ndiff q [before fk]:    \n", (q-q2))
        print("\nparam2 [before fk]:    \n", param2)
        print("\ndiff param [before fk]:\n", (param-param2))

        kp2_in_ext = Keypoints()
        model.forward_kinematics(q2, param2, kp2_in_ext)
        
        # Compute distance between keypoints computed by forward kinematics
        diff_in_ext = Keypoints()
        kpt_distance = Keypoints.keypoint_distance(kp_in_ext, kp2_in_ext, diff_in_ext)

        # Compute distance between configuration and parameter vectors
        q_distance = np.linalg.norm(q-q2)
        param_distance = np.linalg.norm(param-param2)

        print("\nq2 [after fk]:         \n", q2)
        print("\ndiff q:                \n", (q-q2))
        print("\nparam2 [after fk]:     \n", param2)
        print("\ndiff param:            \n", (param-param2)) 

        print("\nkp:                    \n", kp_in_ext.to_string())
        print("\nkp2:                   \n", kp2_in_ext.to_string())
        print("\nkp diff:               \n", diff_in_ext.to_string())
        print("\nkeypoint distance:     \n", kpt_distance)
        print("\nconfiguration distance:\n", q_distance)
        print("\nparam distance:        \n", param_distance, "\n")

        assert kpt_distance < 1e-8,   "keypoint distance is greater than threshold"
        assert q_distance < 1e-8,     "configuration difference is greater than threshold"
        assert param_distance < 1e-8, "param difference is greater than threshold"
       
    end_time = time.time()
    print(f"\nTotal execution time: {(end_time - start_time):.3f} seconds")


def main():
    test_fk()
    test_kinematics()


if __name__ == '__main__':
    main()