#include <chrono>
#include <human_model/human_model.hpp>

int main(int argc, char *argv[])
{
  double shoulder_distance  = 0.3;
  double chest_hip_distance = 0.4;
  double hip_distance       = 0.25;

  double upper_arm_length = 0.3;
  double lower_arm_length = 0.3;
  double upper_leg_length = 0.35;
  double lower_leg_length = 0.4;
  double head_distance    = 0.4;

  int n_dof = 7+3+4*4+2;
  int n_param = 3+2+2+1;
  Eigen::VectorXd q(n_dof);
  Eigen::VectorXd param(n_param);

  // Initialize joint limits
  // (Vector Initialization: The vector qbounds is initialized with n_dof elements,
  // each set to human_model::JointLimits(-M_PI, M_PI))
  std::vector<human_model::JointLimits> qbounds(n_dof,
                                                human_model::JointLimits(-M_PI, M_PI));

  // Set the shoulder rot y joint limits
  qbounds[12]=human_model::JointLimits(-M_PI/2,M_PI/2); // right shoulder
  qbounds[16]=human_model::JointLimits(-M_PI/2,M_PI/2); // left shoulder
  qbounds[20]=human_model::JointLimits(-M_PI/2,M_PI/2); // right hip
  qbounds[24]=human_model::JointLimits(-M_PI/2,M_PI/2); // left hip 

  // Start timing
  auto start = std::chrono::high_resolution_clock::now();

  // Test the fk and ik functions
  for (size_t idx=0; idx<1e4; idx++)
  {
    std::cout << std::endl << "=======================================";
    std::cout << "=======================================" << std::endl;
    std::cout << "idx: " << idx << std::endl;

    q.setRandom();
    q.block(3,0,4,1)/=q.block(3,0,4,1).norm(); // normalize the quaternion
    // If the scalar part of the quaternion is negative,
    // multiply by -1 to ensure consistent representation
    if (q(6, 0) < 0) {
      q.block(3,0,4,1) *= -1.0;
    }

    param(0)=shoulder_distance;
    param(1)=chest_hip_distance;
    param(2)=hip_distance;
    param(3)=upper_arm_length;
    param(4)=lower_arm_length;
    param(5)=upper_leg_length;
    param(6)=lower_leg_length;
    param(7)=head_distance;

    std::cout << "q [original]           : \n" << q.transpose() << std::endl;
    std::cout << "\nparam [original]       : \n" << param.transpose() << std::endl;

    human_model::keypoints kp_in_ext;
    human_model::Human28DOF model;

    human_model::Human28DOF::fk(q,param,kp_in_ext);

    Eigen::VectorXd q2(7+3+4*4+2);
    Eigen::VectorXd param2(3+2+2+1);
    human_model::keypoints kp2_in_ext;
    human_model::keypoints diff_in_ext;

    human_model::Human28DOF::ik(kp_in_ext,qbounds,q2,param2);

    std::cout << "\nq2 [before fk]           : \n" << q2.transpose() << std::endl;
    std::cout << "\ndiff q [before fk]       : \n" << (q-q2).transpose() << std::endl;
    std::cout << "\nparam2 [before fk]       : \n" << param2.transpose() << std::endl;
    std::cout << "\ndiff param [before fk]   : \n" << (param-param2).norm() << std::endl;

    human_model::Human28DOF::fk(q2,param2,kp2_in_ext);
    
    double kpt_distance = human_model::keypoints::keypointDistance(kp_in_ext,kp2_in_ext,diff_in_ext);
    double q_distance = (q-q2).norm();
    double param_distance = (param-param2).norm();

    std::cout << "\nq2 [after fk]           : \n" << q2.transpose() << std::endl;
    std::cout << "\ndiff q      : \n" << (q-q2).transpose() << std::endl;
    std::cout << "\nparam2 [after fk]       : \n" << param2.transpose() << std::endl;
    std::cout << "\ndiff param  : \n" << (param-param2).transpose() << std::endl;

    std::cout << "\nkp          : \n" << kp_in_ext << std::endl;
    std::cout << "\nkp2         : \n" << kp2_in_ext << std::endl;
    std::cout << "\nkp diff     : \n" << diff_in_ext << std::endl;
    std::cout << "\nkeypoint distance: \n" << kpt_distance << std::endl;
    std::cout << "\nconfiguration distance: \n" << q_distance << std::endl;
    std::cout << "\nparam distance: \n" << param_distance << std::endl << std::endl;

    assert(("keypoint distance is greater than threshold",kpt_distance<1e-8));
    assert(("configuration difference is greater than threshold",q_distance<1e-8));
    assert(("param difference is greater than threshold",param_distance<1e-8));
  }

  // End timing
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  std::cout << "Total execution time: " << elapsed.count() << " seconds" << std::endl;

  return 0;
}
