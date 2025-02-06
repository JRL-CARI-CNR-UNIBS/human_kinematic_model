#include <gtest/gtest.h>
#include <Eigen/Dense>
#include <chrono>
#include <vector>
#include <random>
#include <human_model/human_model.hpp>

#define NUM_TESTS 1e4

class HumanKinematicModelTest : public ::testing::Test {
protected:
  // Member variables
  double shoulder_distance;
  double chest_hip_distance;
  double hip_distance;
  double upper_arm_length;  
  double lower_arm_length;  
  double upper_leg_length;
  double lower_leg_length;  
  double head_distance;     

  size_t n_dof;
  size_t n_param;
  Eigen::VectorXd q;
  Eigen::VectorXd q2;
  Eigen::VectorXd param;
  Eigen::VectorXd param2;
  std::vector<human_model::JointLimits> qbounds;
  human_model::keypoints kp_in_ext;
  human_model::keypoints kp2_in_ext;
  human_model::keypoints diff_in_ext;

  std::random_device rd;  // Seed for the random number engine
  std::mt19937 gen; // Standard mersenne_twister_engine seeded with rd()

  // Constructor
  HumanKinematicModelTest()
    : shoulder_distance(0.3),
      chest_hip_distance(0.4),
      hip_distance(0.25),
      upper_arm_length(0.3),
      lower_arm_length(0.3),
      upper_leg_length(0.35),
      lower_leg_length(0.4),
      head_distance(0.4),
      n_dof(28),
      n_param(8),
      q(n_dof),
      q2(n_dof),
      param(n_param),
      param2(n_param),
      gen(rd()) {}

  // SetUp function
  void SetUp() override {
    // Initialize class members
    q.setZero();
    q2.setZero();
    param << shoulder_distance, chest_hip_distance, hip_distance, upper_arm_length,
             lower_arm_length, upper_leg_length, lower_leg_length, head_distance;
    param2.setZero();
    std::map<std::string, Eigen::Vector3d> init_keypoints({
      {"head", Eigen::Vector3d::Zero()},
      {"left_shoulder", Eigen::Vector3d::Zero()},
      {"left_elbow", Eigen::Vector3d::Zero()},
      {"left_wrist", Eigen::Vector3d::Zero()},
      {"left_hip", Eigen::Vector3d::Zero()},
      {"left_knee", Eigen::Vector3d::Zero()},
      {"left_ankle", Eigen::Vector3d::Zero()},
      {"right_shoulder", Eigen::Vector3d::Zero()},
      {"right_elbow", Eigen::Vector3d::Zero()},
      {"right_wrist", Eigen::Vector3d::Zero()},
      {"right_hip", Eigen::Vector3d::Zero()},
      {"right_knee", Eigen::Vector3d::Zero()},
      {"right_ankle", Eigen::Vector3d::Zero()}
    });
    kp_in_ext.set_keypoints(init_keypoints);
    kp2_in_ext.set_keypoints(init_keypoints);
    diff_in_ext.set_keypoints(init_keypoints);

    // Initialize joint limits
    // (Vector Initialization: The vector qbounds is initialized with n_dof elements,
    // each set to human_model::JointLimits(-M_PI, M_PI) -> 1 full rotation)
    qbounds = std::vector<human_model::JointLimits>(n_dof, human_model::JointLimits(-M_PI, M_PI));

    // Set the chest translation and rotation limits -> NOT USED BY THE HUMAN MODEL
    qbounds[0] = human_model::JointLimits( 0.0, 2.0);               // q0_chest_x
    qbounds[1] = human_model::JointLimits(-2.5, 2.5);               // q1_chest_y
    qbounds[2] = human_model::JointLimits( 0.8, 1.8);               // q2_chest_z
    qbounds[3] = human_model::JointLimits(-1.0, 1.0);               // q3_chest_qx
    qbounds[4] = human_model::JointLimits(-1.0, 1.0);               // q4_chest_qy
    qbounds[5] = human_model::JointLimits(-1.0, 1.0);               // q5_chest_qz
    qbounds[6] = human_model::JointLimits(-1.0, 1.0);               // q6_chest_qw
    
    // Set the shoulder axis limits    
    qbounds[7] = human_model::JointLimits(-M_PI / 4, M_PI / 4);     // q7_shoulder_rotx
    
    // Set the hip axis limits    
    qbounds[8] = human_model::JointLimits(-M_PI / 2, M_PI / 2);     // q8_hip_rotz
    qbounds[9] = human_model::JointLimits(-M_PI / 4, M_PI / 4);     // q9_hip_rotx
 
    // Set the right and left shoulder limits 
    qbounds[10] = human_model::JointLimits(-M_PI, M_PI/2 );         // q10_rshoulder_rotz
    qbounds[11] = human_model::JointLimits(-M_PI, 0.0);             // q11_rshoulder_rotx
    qbounds[12] = human_model::JointLimits(-0.75*M_PI, 0.75*M_PI);  // q12_rshoulder_roty
    qbounds[14] = human_model::JointLimits(-M_PI, M_PI/2 );         // q14_lshoulder_rotz
    qbounds[15] = human_model::JointLimits(-M_PI, 0.0);             // q15_lshoulder_rotx
    qbounds[16] = human_model::JointLimits(-0.75*M_PI, 0.75*M_PI);  // q16_lshoulder_roty
    
    // Set the right and left elbow limits 
    qbounds[13] = human_model::JointLimits(-M_PI, 0.0);             // q13_relbow_rotz
    qbounds[17] = human_model::JointLimits(-M_PI, 0.0);             // q17_lelbow_rotz
 
    // Set the right and left hip limits 
    qbounds[18] = human_model::JointLimits(-M_PI/4, M_PI *.75);     // q18_rhip_rotz
    qbounds[19] = human_model::JointLimits(-M_PI / 2, 0.0);         // q19_rhip_rotx
    qbounds[20] = human_model::JointLimits(0.0, M_PI / 2);          // q20_rhip_roty
    qbounds[22] = human_model::JointLimits(-M_PI/4, M_PI *.75);     // q22_lhip_rotz
    qbounds[23] = human_model::JointLimits(-M_PI, 0.0);             // q23_lhip_rotx
    qbounds[24] = human_model::JointLimits(0.0, M_PI / 2);          // q24_lhip_roty
 
    // Set the right and left knee limits 
    qbounds[21] = human_model::JointLimits(0.0, M_PI);              // q21_rknee_rotz
    qbounds[25] = human_model::JointLimits(0.0, M_PI);              // q25_lknee_rotz

    // Set the head limits
    qbounds[26] = human_model::JointLimits(-M_PI / 2, M_PI / 2);    // q26_head_rotx
    qbounds[27] = human_model::JointLimits(-M_PI / 2, M_PI / 2);    // q27_head_roty
  }
};


// Define a test case
TEST_F(HumanKinematicModelTest, TestFkIk) {
  // Start timing
  auto start = std::chrono::high_resolution_clock::now();

  // Test the fk and ik functions
  for (size_t idx=0; idx<NUM_TESTS; idx++)
  {
    std::cout << std::endl << "=======================================";
    std::cout << "=======================================" << std::endl;
    std::cout << "idx: " << idx+1 << std::endl;

    // Set a random configuration
    for (size_t i=0; i<n_dof; i++)
    {
      std::uniform_real_distribution<> dis(qbounds[i].min_, qbounds[i].max_);
      q[i] = dis(gen);
    }

    // // Set plausible values for q
    // q << 0.507435, -1.38239, 1.71223, 0.210431, -0.570813, -0.542813, 0.579003,
    //   -0.345606, 0.260773, 2.86399, -0.703824, 2.88584, 0.30198, -0.77536,
    //   -2.57424, 2.84838, 0.120719, 2.69964, 1.92541, -1.66947, -1.47958,
    //   -0.0824709, 2.08016, 2.0916, 0.733797, -1.67655, 0.263086, -2.86487;

    // Normalize the quaternion
    q.block(3,0,4,1)/=q.block(3,0,4,1).norm();

    // If the scalar part of the quaternion is negative,
    // multiply by -1 to ensure consistent representation
    if (q(6, 0) < 0) {
      q.block(3,0,4,1) *= -1.0;
    }

    std::cout << "q [original]           : \n" << q.transpose() << std::endl;
    std::cout << "\nparam [original]       : \n" << param.transpose() << std::endl;

    // Compute the forward kinematics and inverse kinematics
    human_model::Human28DOF::fk(q,param,kp_in_ext);
    human_model::Human28DOF::ik(kp_in_ext,qbounds,q2,param2);

    std::cout << "\nq2 [before fk]           : \n" << q2.transpose() << std::endl;
    std::cout << "\ndiff q [before fk]       : \n" << (q-q2).transpose() << std::endl;
    std::cout << "\nparam2 [before fk]       : \n" << param2.transpose() << std::endl;
    std::cout << "\ndiff param [before fk]   : \n" << (param-param2).norm() << std::endl;

    // Compute again the forward kinematics
    human_model::Human28DOF::fk(q2,param2,kp2_in_ext);

    Eigen::VectorXd diff_q = q - q2;


    std::cout << "\nq  [after fk]  0-9 : \n" << q.head(10).transpose() << std::endl;
    std::cout << "\nq2 [after fk]      : \n" << q2.head(10).transpose() << std::endl;
    std::cout << "\ndiff q             : \n" << diff_q.head(10).transpose() << std::endl;
    
    for (size_t iarm=0;iarm<4;iarm++)
    {
      std::cout << "\nq  [after fk]  arm : \n" << q.block(10+iarm*4,0,4,1).transpose() << std::endl;
      std::cout << "\nq2 [after fk]      : \n" << q2.block(10+iarm*4,0,4,1).transpose() << std::endl;
      std::cout << "\ndiff q             : \n" << diff_q.block(10+iarm*4,0,4,1).transpose() << std::endl;
    }
    std::cout << "\nq  [after fk]  26-27 : \n" << q.tail(2).transpose() << std::endl;
    std::cout << "\nq2 [after fk]        : \n" << q2.tail(2).transpose() << std::endl;
    std::cout << "\ndiff q               : \n" << diff_q.tail(2).transpose() << std::endl;
    
    
    std::cout << "\nparam2 [after fk]       : \n" << param2.transpose() << std::endl;
    std::cout << "\ndiff param  : \n" << (param-param2).transpose() << std::endl;
    
    // Compute the keypoint distance, configuration distance, and param distance
    double kpt_distance = human_model::keypoints::keypointDistance(kp_in_ext,kp2_in_ext,diff_in_ext);
    double q_distance = (q-q2).norm();
    double param_distance = (param-param2).norm();

    std::cout << "\nkp          : \n" << kp_in_ext << std::endl;
    std::cout << "\nkp2         : \n" << kp2_in_ext << std::endl;
    std::cout << "\nkp diff     : \n" << diff_in_ext << std::endl;
    std::cout << "\nkeypoint distance: \n" << kpt_distance << std::endl;
    std::cout << "\nconfiguration distance: \n" << q_distance << std::endl;
    std::cout << "\nparam distance: \n" << param_distance << std::endl << std::endl;

    // Check if the keypoint distance, configuration distance, and param distance are less than the threshold
    EXPECT_LT(kpt_distance, 1e-8) << "keypoint distance is greater than threshold";
    EXPECT_LT(q_distance, 1e-8) << "configuration difference is greater than threshold";
    EXPECT_LT(param_distance, 1e-8) << "param difference is greater than threshold";
  }

  // End timing
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Total execution time: " << elapsed.count() << " seconds" << std::endl;
}


int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}