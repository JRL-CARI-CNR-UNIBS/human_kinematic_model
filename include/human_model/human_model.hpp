/*
Copyright (c) 2022, JRL-CARI CNR-STIIMA/UNIBS
Manuel Beschi manuel.beschi@unibs.it
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <rdyn_core/primitives.h>




namespace human_model
{



struct keypoints
{
  Eigen::Vector3d head;

  Eigen::Vector3d left_shoulder;
  Eigen::Vector3d left_elbow;
  Eigen::Vector3d left_wrist;

  Eigen::Vector3d left_hip;
  Eigen::Vector3d left_knee;
  Eigen::Vector3d left_ankle;



  Eigen::Vector3d right_shoulder;
  Eigen::Vector3d right_elbow;
  Eigen::Vector3d right_wrist;

  Eigen::Vector3d right_hip;
  Eigen::Vector3d right_knee;
  Eigen::Vector3d right_ankle;

};


class Human28DOF
{
protected:
  Eigen::VectorXd configurations_;
  Eigen::VectorXd velocities_;


public:
  Human28DOF(){};

  /**
   * @brief inverse kinematics from frames to coordinates
   * @param measures_in_ext: measures in external frame (camera, world, ..)
   * @param configuration: human configuration
   */
  static void ik(const keypoints& measures_in_ext,
                 Eigen::VectorXd& configuration,
                 Eigen::VectorXd& param);
  static void fk(const Eigen::VectorXd& configuration,
                 const Eigen::VectorXd& param,
                 keypoints& kp_in_ext) ;

  static double keypointDistance(const keypoints& kp1_in_ext,
                               const keypoints& kp2_in_ext,
                               keypoints& diff_in_ext);

  static void trunckIk(const keypoints& measures_in_ext,
                        Eigen::VectorXd& q,
                        Eigen::VectorXd& param);

  static void trunckFk(const Eigen::VectorXd& q,
                       const Eigen::VectorXd& param,
                       Eigen::Affine3d &T_ext_rshoulder,
                       Eigen::Affine3d &T_ext_lshoulder,
                       Eigen::Affine3d &T_ext_rhip,
                       Eigen::Affine3d &T_ext_lhip,
                       Eigen::Affine3d &T_ext_chest);

  static void headFk(const Eigen::VectorXd& q,
                     const Eigen::VectorXd& param,
                     const Eigen::Affine3d& T_ext_chest,
                     Eigen::Affine3d &T_ext_head);

  static void headIk(const keypoints& measures_in_ext,
                     const Eigen::Affine3d& T_ext_chest,
                     Eigen::VectorXd& q,
                     Eigen::VectorXd& param);


  static void rightLimbFk(  const Eigen::VectorXd& qarm,
                              const Eigen::VectorXd& param,
                              Eigen::Vector3d& elbow_in_limb,
                              Eigen::Vector3d& wrist_in_limb);


  static void leftLimbFk(  const Eigen::VectorXd& qarm,
                             const Eigen::VectorXd& param,
                             Eigen::Vector3d& elbow_in_limb,
                             Eigen::Vector3d& wrist_in_limb);

  static void rightLimbIk(const Eigen::Vector3d& elbow_in_limb,
                             const Eigen::Vector3d& wrist_in_limb,
                            const Eigen::VectorXd& param,
                              Eigen::VectorXd& qarm);


  static void leftLimbIk(const Eigen::Vector3d& elbow_in_limb,
                           const Eigen::Vector3d& wrist_in_limb,
                           const Eigen::VectorXd& param,
                              Eigen::VectorXd& qarm);

  static void print(const Eigen::VectorXd& q,
                    const Eigen::VectorXd& param);
  friend std::ostream& operator<<(std::ostream& os, const keypoints& keypoints);

};

std::ostream& operator<<(std::ostream& os, const keypoints& keypoints);
}  // end namespace human_model
