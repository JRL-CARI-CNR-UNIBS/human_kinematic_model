// Speed test of the native C++ fk -> ik -> fk loop (as in the README), without the prints of
// test_fk_ik.cpp. Configurations are drawn uniformly within the default joint limits, as in the gtest.
//
// Usage: benchmark_fk_ik [n_configurations=10000] [seed=0]
// Output (parsed by test/python/benchmark_jax.py):
//   n_configurations <n>
//   total_seconds <seconds for the whole loop>
//   max_keypoint_error <max keypoint distance between the two fk calls>

#include <chrono>
#include <cstdlib>
#include <iostream>
#include <random>
#include <vector>
#include <human_model/human_model.hpp>

int main(int argc, char** argv)
{
  const size_t n_configurations = argc > 1 ? std::strtoul(argv[1], nullptr, 10) : 10000;
  const unsigned seed = argc > 2 ? std::strtoul(argv[2], nullptr, 10) : 0;
  const size_t n_dof = 28;

  Eigen::VectorXd param(8);
  param << 0.3, 0.4, 0.25, 0.3, 0.3, 0.35, 0.4, 0.4;

  std::vector<human_model::JointLimits> qbounds;
  human_model::Human28DOF::setDefaultJointLimits(qbounds);

  // Draw the configurations before timing
  std::mt19937 gen(seed);
  std::vector<Eigen::VectorXd> configurations(n_configurations, Eigen::VectorXd(n_dof));
  for (Eigen::VectorXd& q : configurations)
  {
    for (size_t i = 0; i < n_dof; i++)
      q(i) = std::uniform_real_distribution<>(qbounds[i].min_, qbounds[i].max_)(gen);
    q.segment(3, 4) /= q.segment(3, 4).norm();
    if (q(6) < 0)
      q.segment(3, 4) *= -1.0;
  }

  human_model::keypoints kp_in_ext, kp2_in_ext, diff_in_ext;
  Eigen::VectorXd q2, param2;
  Eigen::Vector4d chest_q_rotated;
  double max_keypoint_error = 0.0;

  auto start = std::chrono::high_resolution_clock::now();
  for (const Eigen::VectorXd& q : configurations)
  {
    human_model::Human28DOF::fk(q, param, kp_in_ext);
    human_model::Human28DOF::ik(kp_in_ext, qbounds, q, q2, param2, chest_q_rotated);
    human_model::Human28DOF::fk(q2, param2, kp2_in_ext);
    // also keeps the compiler from optimizing the loop away
    max_keypoint_error = std::max(max_keypoint_error,
                                  human_model::keypoints::keypointDistance(kp_in_ext, kp2_in_ext, diff_in_ext));
  }
  std::chrono::duration<double> elapsed = std::chrono::high_resolution_clock::now() - start;

  std::cout << "n_configurations " << n_configurations << std::endl;
  std::cout << "total_seconds " << elapsed.count() << std::endl;
  std::cout << "max_keypoint_error " << max_keypoint_error << std::endl;
  return 0;
}
