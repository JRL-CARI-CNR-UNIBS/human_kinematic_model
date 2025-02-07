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

#include <human_model/human_model.hpp>

namespace human_model
{

bool Human28DOF::updateIfCloser(const Eigen::VectorXd& qarm_temp,
                                const Eigen::VectorXd& qarm_previous,
                                Eigen::VectorXd& qarm,
                                double& q_distance)
{
  bool is_closer(false);
  double q_distance_temp = (qarm_temp - qarm_previous).norm();
  if (q_distance_temp < q_distance)
  {
    qarm = qarm_temp;
    q_distance = q_distance_temp;
    is_closer = true;
  }
  return is_closer;
}


void Human28DOF::computeWristIn2(const Eigen::Vector2d& qshoulder,
                                 const Eigen::Vector3d& wrist_in_limb,
                                 Eigen::Vector3d& wrist_in_2)
{
  Eigen::AngleAxisd rot01(qshoulder(0), Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot12(qshoulder(1), Eigen::Vector3d::UnitX());

  Eigen::Affine3d T01(rot01);
  Eigen::Affine3d T12(rot12);

  Eigen::Affine3d T02=T01*T12;

  wrist_in_2=T02.inverse()*wrist_in_limb;
}


bool Human28DOF::shoulderIk(const Eigen::Vector3d& elbow_in_limb,
                            const std::vector<JointLimits>& qshoulder_bounds,
                            bool first_solution,
                            Eigen::Vector2d& qshoulder)
{
  // Configuration
  double& q1=qshoulder(0);  // shoulder rot z
  double& q2=qshoulder(1);  // shoulder rot x

  // Joint limits
  const double& q1min=qshoulder_bounds[0].min_; // shoulder rot z lower bound
  const double& q1max=qshoulder_bounds[0].max_; // shoulder rot z upper bound
  const double& q2min=qshoulder_bounds[1].min_; // shoulder rot x lower bound
  const double& q2max=qshoulder_bounds[1].max_; // shoulder rot x upper bound

  // q1: SHOULDER ROT Z
  if (first_solution)
    // Solution 1: Hypothesis -PI/2<q2<PI/2 (cos(q2)>0)
    q1=std::atan2(-elbow_in_limb(0),elbow_in_limb(1));
  else
    // Solution 2: Hypothesis -PI<q2<-PI/2 or PI/2<q2<PI (cos(q2)<0)
    q1=std::atan2(elbow_in_limb(0),-elbow_in_limb(1));
    
  // q2: SHOULDER ROT X
  if (std::abs(std::sin(q1))>0.5)
    q2=std::atan2(elbow_in_limb(2),-elbow_in_limb(0)/std::sin(q1));
  else
    q2=std::atan2(elbow_in_limb(2),elbow_in_limb(1)/std::cos(q1));

  // check if the solution is within the joint limits
  bool valid_solution=(q1>q1min && q1<q1max && q2>q2min && q2<q2max);
  
  // check if the solution is valid
  if (first_solution)
    // (cos(q2)>0)
    valid_solution=valid_solution && (std::cos(q2)>0);
  else
    // (cos(q2)<0)
    valid_solution=valid_solution && (std::cos(q2)<0);

  // Assign nan if the solution is not valid
  q1 = valid_solution ? q1 : std::nan("");
  q2 = valid_solution ? q2 : std::nan("");

  return valid_solution;
}


bool Human28DOF::elbowIk(const Eigen::Vector3d& wrist_in_limb,
                         const double& upper_arm_length,
                         const std::vector<JointLimits>& qelbow_bounds,
                         bool first_solution,
                         Eigen::Vector2d& qelbow)
{
  // Configuration
  double& q3=qelbow(0);  // shoulder rot y
  double& q5=qelbow(1);  // elbow rot z

  // Parameters
  const double& q4=upper_arm_length;  // upper arm length

  // Joint limits
  const double& q3min=qelbow_bounds[0].min_; // shoulder rot y lower bound
  const double& q3max=qelbow_bounds[0].max_; // shoulder rot y upper bound
  const double& q5min=qelbow_bounds[1].min_; // elbow rot z lower bound
  const double& q5max=qelbow_bounds[1].max_; // elbow rot z upper bound

  double q6cosq5 = wrist_in_limb(1)-q4;

  // q3: SHOULDER ROT Y
  if (first_solution)
    // Solution 1: Hypothesis  0<q5<PI (sin(q5)>0)
    q3=std::atan2(wrist_in_limb(2),-wrist_in_limb(0));
  else
    // Solution 2: Hypothesis -PI<q5<0 (sin(q5)<0)
    q3=std::atan2(-wrist_in_limb(2),wrist_in_limb(0));

  // q5: ELBOW ROT Z 
  if (std::abs(std::sin(q3))>0.5)
    q5=std::atan2(wrist_in_limb(2)/std::sin(q3),q6cosq5);
  else
    q5=std::atan2(-wrist_in_limb(0)/std::cos(q3),q6cosq5);

  // check if the solution is within the joint limits
  bool valid_solution=(q3>q3min && q3<q3max && q5>q5min && q5<q5max);

  // check if the solution is valid
  if (first_solution)
    // (sin(q5)>0)
    valid_solution=valid_solution && (std::sin(q5)>0);
  else
    // (sin(q5)<0)
    valid_solution=valid_solution && (std::sin(q5)<0);

  // Assign nan if the solution is not valid
  q3 = valid_solution ? q3 : std::nan("");
  q5 = valid_solution ? q5 : std::nan("");

  return valid_solution;
}


void Human28DOF::rightLimbIk(const Eigen::Vector3d& elbow_in_limb,
                             const Eigen::Vector3d& wrist_in_limb,
                             const Eigen::VectorXd& param,
                             const std::vector<JointLimits>& qarm_bounds,
                             const Eigen::VectorXd& qarm_previous,
                             Eigen::VectorXd& qarm)
{
  // Configuration
  double& q1=qarm(0);  // shoulder rot z
  double& q2=qarm(1);  // shoulder rot x
  double& q3=qarm(2);  // shoulder rot y
  double& q5=qarm(3);  // elbow rot z

  // Parameters
  const double& q4=param(0);  // upper arm length
  const double& q6=param(1);  // lower arm length

  // Joint limits
  std::vector<JointLimits> q_shoulder_bounds = {
    qarm_bounds[0], // shoulder rot z
    qarm_bounds[1]  // shoulder rot x
  };

  std::vector<JointLimits> q_elbow_bounds = {
    qarm_bounds[2], // shoulder rot y 
    qarm_bounds[3]  // elbow rot z
  };

  // ### ELBOW IN LIMB FRAME ###
  Eigen::Vector2d q_a, q_b;
  bool valid_sol_a=shoulderIk(elbow_in_limb,q_shoulder_bounds,true,q_a);  // first solution
  bool valid_sol_b=shoulderIk(elbow_in_limb,q_shoulder_bounds,false,q_b); // second solution

  // Throw exception if there is no solution with the
  // SHOULDER ROT Z and SHOULDER ROT X within the limits
  if (!valid_sol_a && !valid_sol_b)
    throw std::runtime_error("No solution for the SHOULDER ROT Z and SHOULDER ROT X within the limits.");
  // ### END ELBOW IN LIMB FRAME ###

  // ### WRIST IN FRAME #2 ###
  Eigen::Vector3d wrist_in_2_a;
  Eigen::Vector3d wrist_in_2_b;
  computeWristIn2(q_a,wrist_in_limb,wrist_in_2_a);
  computeWristIn2(q_b,wrist_in_limb,wrist_in_2_b);

  // Compute the two solutions for the ELBOW for each of the two solutions for the SHOULDER
  Eigen::Vector2d q_elbow_aa, q_elbow_ab, q_elbow_ba, q_elbow_bb;
  bool valid_sol_aa=elbowIk(wrist_in_2_a,q4,q_elbow_bounds,true,q_elbow_aa);  // first solution for q_a
  bool valid_sol_ab=elbowIk(wrist_in_2_a,q4,q_elbow_bounds,false,q_elbow_ab); // second solution for q_a
  bool valid_sol_ba=elbowIk(wrist_in_2_b,q4,q_elbow_bounds,true,q_elbow_ba);  // first solution for q_b
  bool valid_sol_bb=elbowIk(wrist_in_2_b,q4,q_elbow_bounds,false,q_elbow_bb); // second solution for q_b
  
  // Throw exception if there is no solution with the
  // SHOULDER ROT Y and ELBOW ROT Z within the limits
  if (!valid_sol_aa && !valid_sol_ab && !valid_sol_ba && !valid_sol_bb)
    throw std::runtime_error("No solution for the SHOULDER ROT Y and ELBOW ROT Z within the limits.");
  // ### END WRIST IN FRAME #2 ###

  // Select q1, q2, q3, q5 based on the valid solutions and the previous configuration
  double q_distance=std::numeric_limits<double>::infinity();
  Eigen::VectorXd qarm_temp(4);
  bool is_closer(false);
  if (valid_sol_aa && valid_sol_a)
  {
    // std::cout << "\tvalid_sol_aa && valid_sol_a" << std::endl;
    qarm_temp << q_a(0), q_a(1), q_elbow_aa(0), q_elbow_aa(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_aa && valid_sol_a" << std::endl;
  }
  if (valid_sol_aa && valid_sol_b)
  {
    // std::cout << "\tvalid_sol_aa && valid_sol_b" << std::endl;
    qarm_temp << q_b(0), q_b(1), q_elbow_aa(0), q_elbow_aa(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_aa && valid_sol_b" << std::endl;
  }
  if (valid_sol_ab && valid_sol_a)
  {
    // std::cout << "\tvalid_sol_ab && valid_sol_a" << std::endl;
    qarm_temp << q_a(0), q_a(1), q_elbow_ab(0), q_elbow_ab(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_ab && valid_sol_a" << std::endl;
  }
  if (valid_sol_ab && valid_sol_b)
  {
    // std::cout << "\tvalid_sol_ab && valid_sol_b" << std::endl;
    qarm_temp << q_b(0), q_b(1), q_elbow_ab(0), q_elbow_ab(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_ab && valid_sol_b" << std::endl;
  }
  if (valid_sol_ba && valid_sol_a)
  {
    // std::cout << "\tvalid_sol_ba && valid_sol_a" << std::endl;
    qarm_temp << q_a(0), q_a(1), q_elbow_ba(0), q_elbow_ba(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_ba && valid_sol_a" << std::endl;
  }
  if (valid_sol_ba && valid_sol_b)
  {
    // std::cout << "\tvalid_sol_ba && valid_sol_b" << std::endl;
    qarm_temp << q_b(0), q_b(1), q_elbow_ba(0), q_elbow_ba(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_ba && valid_sol_b" << std::endl;
  }
  if (valid_sol_bb && valid_sol_a)
  {
    // std::cout << "\tvalid_sol_bb && valid_sol_a" << std::endl;
    qarm_temp << q_a(0), q_a(1), q_elbow_bb(0), q_elbow_bb(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_bb && valid_sol_a" << std::endl;
  }
  if (valid_sol_bb && valid_sol_b)
  {
    // std::cout << "\tvalid_sol_bb && valid_sol_b" << std::endl;
    qarm_temp << q_b(0), q_b(1), q_elbow_bb(0), q_elbow_bb(1);
    is_closer=updateIfCloser(qarm_temp,qarm_previous,qarm,q_distance);
    // if (is_closer)
      // std::cout << "\t\tselected valid_sol_bb && valid_sol_b" << std::endl;
  }
  // Throw an exception if there is no valid solution
  if (!valid_sol_aa && !valid_sol_ab && !valid_sol_ba && !valid_sol_bb)
  {
    q1, q2, q3, q5 = std::nan("");
    throw std::runtime_error("No solution for the SHOULDER ROT Z, SHOULDER ROT X, SHOULDER ROT Y, and ELBOW ROT Z within the limits.");
  }
}


void Human28DOF::leftLimbIk(const Eigen::Vector3d& elbow_in_limb,
                            const Eigen::Vector3d& wrist_in_limb,
                            const Eigen::VectorXd& param,
                            const std::vector<JointLimits>& qarm_bounds,
                            const Eigen::VectorXd& qarm_previous,
                            Eigen::VectorXd& qarm)
{
  Eigen::Vector3d mirror_elbow_in_limb=elbow_in_limb;
  Eigen::Vector3d mirror_wrist_in_limb=wrist_in_limb;
  mirror_elbow_in_limb(2)*=-1.0;
  mirror_wrist_in_limb(2)*=-1.0;
  rightLimbIk(mirror_elbow_in_limb,mirror_wrist_in_limb,param,qarm_bounds,qarm_previous,qarm);
  return;
}


void Human28DOF::rightLimbFk(const Eigen::VectorXd& qarm,
                             const Eigen::VectorXd& param,
                             Eigen::Vector3d& elbow_in_limb,
                             Eigen::Vector3d& wrist_in_limb)
{
  const double& q1=qarm(0);   // shoulder rot z
  const double& q2=qarm(1);   // shoulder rot x
  const double& q3=qarm(2);   // shoulder rot y
  const double& q5=qarm(3);   // elbow rot z
  const double& q4=param(0);  // upper arm length
  const double& q6=param(1);  // lower arm length

  Eigen::AngleAxisd rot01(q1,Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T01;
  T01=rot01;

  Eigen::AngleAxisd rot12(q2,Eigen::Vector3d::UnitX());
  Eigen::Affine3d T12;
  T12=rot12;

  Eigen::AngleAxisd rot23(q3,Eigen::Vector3d::UnitY());
  Eigen::Affine3d T23;
  T23=rot23;

  Eigen::Affine3d T34; // translation along y
  T34.setIdentity();
  T34.translation()(1)=q4;

  Eigen::AngleAxisd rot45(q5,Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T45;
  T45=rot45;

  Eigen::Affine3d T56; // translation along y
  T56.setIdentity();
  T56.translation()(1)=q6;

  Eigen::Affine3d T04=T01*T12*T23*T34;
  Eigen::Affine3d T06=T04*T45*T56;

  elbow_in_limb=T04.translation();
  wrist_in_limb=T06.translation();
}

void Human28DOF::rightLimbFk_tfs(const Eigen::VectorXd& qarm,
                                 const Eigen::VectorXd& param,
                                 Eigen::Affine3d& T_limb_shoulderRotated,
                                 Eigen::Affine3d& T_limb_elbow,
                                 Eigen::Affine3d& T_limb_wrist)
{
  const double& q1=qarm(0);   // shoulder rot z
  const double& q2=qarm(1);   // shoulder rot x
  const double& q3=qarm(2);   // shoulder rot y
  const double& q5=qarm(3);   // elbow rot z
  const double& q4=param(0);  // upper arm length
  const double& q6=param(1);  // lower arm length

  Eigen::AngleAxisd rot01(q1,Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T01;
  T01=rot01;

  Eigen::AngleAxisd rot12(q2,Eigen::Vector3d::UnitX());
  Eigen::Affine3d T12;
  T12=rot12;

  Eigen::AngleAxisd rot23(q3,Eigen::Vector3d::UnitY());
  Eigen::Affine3d T23;
  T23=rot23;

  T_limb_shoulderRotated = T01*T12*T23;

  Eigen::Affine3d T34; // translation along y
  T34.setIdentity();
  T34.translation()(1)=q4;

  Eigen::AngleAxisd rot45(q5,Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T45;
  T45=rot45;

  Eigen::Affine3d T56; // translation along y
  T56.setIdentity();
  T56.translation()(1)=q6;

  T_limb_elbow=T_limb_shoulderRotated*T34;
  T_limb_wrist=T_limb_elbow*T45*T56;
}


void Human28DOF::leftLimbFk(const Eigen::VectorXd& qarm,
                            const Eigen::VectorXd& param,
                            Eigen::Vector3d& elbow_in_limb,
                            Eigen::Vector3d& wrist_in_limb)
{
  Human28DOF::rightLimbFk(qarm,param,elbow_in_limb,wrist_in_limb);
  elbow_in_limb(2)*=-1.0;
  wrist_in_limb(2)*=-1.0;
}

void Human28DOF::leftLimbFk_tfs(const Eigen::VectorXd& qarm,
                                const Eigen::VectorXd& param,
                                Eigen::Affine3d& T_limb_shoulderRotated,
                                Eigen::Affine3d& T_limb_elbow,
                                Eigen::Affine3d& T_limb_wrist)
{
  Human28DOF::rightLimbFk_tfs(qarm,param,T_limb_shoulderRotated,T_limb_elbow,T_limb_wrist);
  // Invert the z coordinate
  T_limb_shoulderRotated.translation()(2)*=-1.0;
  T_limb_elbow.translation()(2)*=-1.0;
  T_limb_wrist.translation()(2)*=-1.0;

}


void Human28DOF::trunkIk(const keypoints& measures_in_ext,
                         const std::vector<JointLimits>& qtrunk_bounds,
                         Eigen::VectorXd& q,
                         Eigen::VectorXd& param)
{
  // Configuration
  q.resize(7+3);
  double& shoulder_rotx=q(7);
  double& hip_rotz=q(8);
  double& hip_rotx=q(9);
  
  // Parameters
  param.resize(3);
  double& shoulder_distance=param(0);
  double& chest_hip_distance=param(1);
  double& hip_distance=param(2);

  // Joint limits
  double shoulder_rotx_min=qtrunk_bounds[0].min_; // shoulder rot x lower bound
  double shoulder_rotx_max=qtrunk_bounds[0].max_; // shoulder rot x upper bound
  double hip_rotz_min=qtrunk_bounds[1].min_; // hip rot z lower bound
  double hip_rotz_max=qtrunk_bounds[1].max_; // hip rot z upper bound
  double hip_rotx_min=qtrunk_bounds[2].min_; // hip rot x lower bound
  double hip_rotx_max=qtrunk_bounds[2].max_; // hip rot x upper bound

  // Compute the versors of the shoulders and the hips
  Eigen::Vector3d shoulder_versor_in_ext=(measures_in_ext.left_shoulder-measures_in_ext.right_shoulder).normalized();
  Eigen::Vector3d hip_versor_in_ext=(measures_in_ext.left_hip-measures_in_ext.right_hip).normalized();

  // Compute the chest reference frame Z
  Eigen::Vector3d upper_chest=0.5*(measures_in_ext.left_shoulder+measures_in_ext.right_shoulder);
  Eigen::Vector3d lower_chest=0.5*(measures_in_ext.left_hip+measures_in_ext.right_hip);
  Eigen::Vector3d chest_z_in_ext=(upper_chest-lower_chest).normalized();

  // Compute the distances between the shoulders and the hips
  shoulder_distance=(measures_in_ext.left_shoulder-measures_in_ext.right_shoulder).norm();
  chest_hip_distance=(upper_chest-lower_chest).norm();
  hip_distance=(measures_in_ext.left_hip-measures_in_ext.right_hip).norm();

  // CHEST REFERENCE FRAME:
  Eigen::Vector3d chest_y_in_ext=shoulder_versor_in_ext-shoulder_versor_in_ext.dot(chest_z_in_ext)*chest_z_in_ext;
  chest_y_in_ext.normalize();
  Eigen::Vector3d chest_x_in_ext=chest_y_in_ext.cross(chest_z_in_ext);

  // Set the columns of chest_rot
  Eigen::Matrix3d chest_rot;
  chest_rot.col(0) = chest_x_in_ext; // frontal direction
  chest_rot.col(1) = chest_y_in_ext; // right to left shoulder
  chest_rot.col(2) = chest_z_in_ext; // vertical axis (lower chest to upper chest)

  // Convert to quaternion
  Eigen::Quaterniond chest_q(chest_rot);

  // If the scalar part of the quaternion is negative,
  // multiply by -1 to ensure consistent representation
  if (chest_q.w() < 0) {
    chest_q.coeffs() *= -1.0;
  }

  Eigen::Affine3d T_ext_chest;
  T_ext_chest=chest_q;
  T_ext_chest.translation()=upper_chest;

  q.block(0,0,3,1)=upper_chest;
  q.block(3,0,4,1)=chest_q.coeffs();

  // shoulder reference frame:
  // T_chest_shoulder = rotx(shoulder_rotx)

  // shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
  Eigen::Vector3d shoulder_versor_in_chest=T_ext_chest.linear().inverse()*shoulder_versor_in_ext;

  shoulder_rotx=std::atan2(shoulder_versor_in_chest(2),shoulder_versor_in_chest(1));
  if (shoulder_rotx<shoulder_rotx_min || shoulder_rotx>shoulder_rotx_max)
    throw std::runtime_error("Shoulder rotation out of bounds.");

  // ### HIP ROT Z and HIP ROT X ###
  Eigen::Vector3d hip_versor_in_chest=T_ext_chest.linear().inverse()*hip_versor_in_ext;
  
  // Solution 1: Hypothesis -PI/2<hip_rotx<PI/2 (cos(hip_rotx)>0)
  double hip_rotz_a=std::atan2(-hip_versor_in_chest(0),hip_versor_in_chest(1));
  double hip_rotx_a;
  if (std::abs(std::sin(hip_rotz_a))>0.5)
    hip_rotx_a=std::atan2(hip_versor_in_chest(2),-hip_versor_in_chest(0)/std::sin(hip_rotz_a));
  else
    hip_rotx_a=std::atan2(hip_versor_in_chest(2),hip_versor_in_chest(1)/std::cos(hip_rotz_a));

  // check if the solution is valid (cos(hip_rotx)>0)
  bool sol_a_valid=(std::cos(hip_rotx_a)>0
    && hip_rotz_a>hip_rotz_min && hip_rotz_a<hip_rotz_max
    && hip_rotx_a>hip_rotx_min && hip_rotx_a<hip_rotx_max);
  hip_rotz = sol_a_valid ? hip_rotz_a : std::nan("");
  hip_rotx = sol_a_valid ? hip_rotx_a : std::nan("");

  if (!sol_a_valid)
  {
    // Solution 2: Hypothesis -PI<hip_rotx<-PI/2 or PI/2<hip_rotx<PI (cos(hip_rotx)<0)
    double hip_rotz_b=std::atan2(hip_versor_in_chest(0),-hip_versor_in_chest(1));
    double hip_rotx_b;
    if (std::abs(std::sin(hip_rotz_b))>0.5)
      hip_rotx_b=std::atan2(hip_versor_in_chest(2),-hip_versor_in_chest(0)/std::sin(hip_rotz_b));
    else
      hip_rotx_b=std::atan2(hip_versor_in_chest(2),hip_versor_in_chest(1)/std::cos(hip_rotz_b));
    
    // check if the solution is valid (cos(hip_rotx)<0)
    bool sol_b_valid=(std::cos(hip_rotx_b)<0
      && hip_rotz_b>hip_rotz_min && hip_rotz_b<hip_rotz_max
      && hip_rotx_b>hip_rotx_min && hip_rotx_b<hip_rotx_max);
    hip_rotz = sol_b_valid ? hip_rotz_b : std::nan("");
    hip_rotx = sol_b_valid ? hip_rotx_b : std::nan("");

    // Throw exception if there is no solution with the
    // HIP ROT Z and HIP ROT X within the limits
    if (!sol_b_valid)
      throw std::runtime_error("No solution for the HIP ROT Z and HIP ROT X within the limits.");
  }
  // ### END HIP ROT Z and HIP ROT X ###
}


void Human28DOF::trunkFk(const Eigen::VectorXd& q,
                         const Eigen::VectorXd& param,
                         Eigen::Affine3d& T_ext_rshoulder,
                         Eigen::Affine3d& T_ext_lshoulder,
                         Eigen::Affine3d& T_ext_rhip,
                         Eigen::Affine3d& T_ext_lhip,
                         Eigen::Affine3d& T_ext_chest)
{
  const double& shoulder_rotx=q(7);
  const double& hip_rotz=q(8);
  const double& hip_rotx=q(9);
  const double& shoulder_distance=param(0);
  const double& chest_hip_distance=param(1);
  const double& hip_distance=param(2);

  // CHEST REFERENCE FRAME:
  // chest orientation (quaternion in scalar-last form)
  Eigen::Quaterniond chest_q;
  chest_q.coeffs()=q.block(3,0,4,1);
  T_ext_chest=chest_q;

  // 3D position of the chest
  T_ext_chest.translation()=q.block(0,0,3,1);

  // ROTATED CHEST REFERENCE FRAME:
  // T_chest_shoulder rotation around x axis of the chest by shoulder_rotx radians
  Eigen::Affine3d T_chest_shoulder;
  T_chest_shoulder=Eigen::AngleAxisd(shoulder_rotx,Eigen::Vector3d::UnitX());

  // RIGHT SHOULDER REFERENCE FRAME:
  // rshoulder_z axis along the line connecting the shoulders
  // rshoulder_y axis downwards
  // rshoulder_x axis aligned to chest_x_axis

  // T_shoulder_rshoulder0: rotation of the axis to have z pointing the the body center
  Eigen::Affine3d T_shoulder_rshoulder0;
  T_shoulder_rshoulder0=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX());

  // T_rshoulder0_rshoulder: translation to have the origin in the shoulder center
  Eigen::Affine3d T_rshoulder0_rshoulder;
  T_rshoulder0_rshoulder.setIdentity();
  T_rshoulder0_rshoulder.translation()=-Eigen::Vector3d::UnitZ()*0.5*shoulder_distance;

  // Combine the two transformations
  // T_shoulder_rshoulder = rotx(pi/2) followed by trans(-0.5*shoulder_distance)
  Eigen::Affine3d T_shoulder_rshoulder=T_shoulder_rshoulder0*T_rshoulder0_rshoulder;

  // Express wrt the external frame
  T_ext_rshoulder=T_ext_chest*T_chest_shoulder*T_shoulder_rshoulder;
  
  // LEFT SHOULDER REFERENCE FRAME:
  // (flipped in the left_limb_ik and left_limb_fk wrt the right one)
  // lshoulder_z axis along the line connecting the shoulders
  // lshoulder_y axis downwards
  // lshoulder_x axis "opposite" to chest_x_axis
 
  // T_shoulder_lshoulder0: same as T_shoulder_rshoulder0
  Eigen::Affine3d T_shoulder_lshoulder0(T_shoulder_rshoulder0);

  // T_lshoulder0_lshoulder: translation to have the origin in the shoulder center
  Eigen::Affine3d T_lshoulder0_lshoulder;
  T_lshoulder0_lshoulder.setIdentity();
  T_lshoulder0_lshoulder.translation()=Eigen::Vector3d::UnitZ()*0.5*shoulder_distance;
  
  // Combine the two transformations
  // T_shoulder_lshoulder = rotx(pi/2) followed by trans(0.5*shoulder_distance)
  Eigen::Affine3d T_shoulder_lshoulder=T_shoulder_lshoulder0*T_lshoulder0_lshoulder;

  // Express wrt the external frame
  T_ext_lshoulder=T_ext_chest*T_chest_shoulder*T_shoulder_lshoulder;


  // DEBUG: Eigen::Vector3d shoulder_versor_in_chest=(T_chest_shoulder).linear()*Eigen::Vector3d::UnitY();


  // HIP REFERENCE FRAME:
  Eigen::Affine3d T_chest_hip0;
  T_chest_hip0.setIdentity();
  T_chest_hip0.translation()(2)=-chest_hip_distance;

  // TWICE-ROTATED HIP REFERENCE FRAME:
  Eigen::Affine3d T_hip0_hip1;  
  T_hip0_hip1=Eigen::AngleAxisd(hip_rotz,Eigen::Vector3d::UnitZ());
  Eigen::Affine3d T_hip1_hip2;
  T_hip1_hip2=Eigen::AngleAxisd(hip_rotx,Eigen::Vector3d::UnitX());

  // DEBUG: Eigen::Vector3d hip_versor_in_chest=(T_chest_hip0*T_chest_hip0*T_hip0_hip1*T_hip1_hip2).linear()*Eigen::Vector3d::UnitY();


  // RIGHT HIP REFERENCE FRAME:
  Eigen::Affine3d T_hip2_rhip3;
  T_hip2_rhip3=Eigen::AngleAxisd(-M_PI*0.5,Eigen::Vector3d::UnitX());

  Eigen::Affine3d T_rhip3_rhip;
  T_rhip3_rhip.setIdentity();
  T_rhip3_rhip.translation()(2)-=0.5*hip_distance;

  // Express wrt the external frame
  T_ext_rhip=T_ext_chest*T_chest_hip0*T_hip0_hip1*T_hip1_hip2*T_hip2_rhip3*T_rhip3_rhip;
  
  // LEFT HIP REFERENCE FRAME:
  // T_hip2_lhip3: same as T_hip2_rhip3
  Eigen::Affine3d T_hip2_lhip3(T_hip2_rhip3);

  Eigen::Affine3d T_lhip3_lhip;
  T_lhip3_lhip.setIdentity();
  T_lhip3_lhip.translation()(2)=0.5*hip_distance;

  // Express wrt the external frame
  T_ext_lhip=T_ext_chest*T_chest_hip0*T_hip0_hip1*T_hip1_hip2*T_hip2_lhip3*T_lhip3_lhip;
}


void Human28DOF::headFk(const Eigen::VectorXd& q,
                        const Eigen::VectorXd& param,
                        const Eigen::Affine3d& T_ext_chest,
                        Eigen::Affine3d &T_ext_head)
{
  const double& q1=q(0);
  const double& q2=q(1);
  const double& distance=param(0);

  Eigen::Affine3d T_chest_head0;
  T_chest_head0=Eigen::AngleAxisd(q1,Eigen::Vector3d::UnitX());
  Eigen::Affine3d T_head0_head1;
  T_head0_head1=Eigen::AngleAxisd(q2,Eigen::Vector3d::UnitY());
  Eigen::Affine3d T_head1_head;
  T_head1_head.setIdentity();
  T_head1_head.translation()=Eigen::Vector3d::UnitZ()*distance;

  T_ext_head=T_ext_chest*T_chest_head0*T_head0_head1*T_head1_head;
}


void Human28DOF::headIk(const keypoints& measures_in_ext,
                        const Eigen::Affine3d& T_ext_chest,
                        const std::vector<JointLimits>& qhead_bounds,
                        Eigen::VectorXd& q,
                        Eigen::VectorXd& param)
{
  // Configuration
  q.resize(2);
  double& q1=q(0);
  double& q2=q(1);

  // Parameters
  param.resize(1);
  Eigen::Vector3d head_in_chest=T_ext_chest.inverse() * measures_in_ext.head;
  param(0)=head_in_chest.norm();

  // Joint limits
  double q1min=qhead_bounds[0].min_; // head rot x lower bound
  double q1max=qhead_bounds[0].max_; // head rot x upper bound
  double q2min=qhead_bounds[1].min_; // head rot y lower bound
  double q2max=qhead_bounds[1].max_; // head rot y upper bound

  // Solution 1: Hypothesis -PI/2<q1<PI/2 (cos(q2)>0)
  double q1a=std::atan2(-head_in_chest(1),head_in_chest(2));
  double q2a;
  if (std::abs(std::sin(q1a))>0.5)
    q2a=std::atan2(head_in_chest(0),-head_in_chest(1)/std::sin(q1a));
  else
    q2a=std::atan2(head_in_chest(0),head_in_chest(2)/std::cos(q1a));

  // check if the solution is valid (cos(q2)>0)
  bool sol_a_valid=(std::cos(q2a)>0 && q1a>q1min && q1a<q1max && q2a>q2min && q2a<q2max);
  q1 = sol_a_valid ? q1a : std::nan("");
  q2 = sol_a_valid ? q2a : std::nan("");

  if (!sol_a_valid)
  {
    // Solution 2: Hypothesis -PI<q1<-PI/2 or PI/2<q1<PI (cos(q2)<0)
    double q1b=std::atan2(head_in_chest(1),-head_in_chest(2));
    double q2b;
    if (std::abs(std::sin(q1b))>0.5)
      q2b=std::atan2(head_in_chest(0),-head_in_chest(1)/std::sin(q1b));
    else
      q2b=std::atan2(head_in_chest(0),head_in_chest(2)/std::cos(q1b));

    // check if the solution is valid (cos(q2)<0)
    bool sol_b_valid=(std::cos(q2b)<0 && q1b>q1min && q1b<q1max && q2b>q2min && q2b<q2max);
    q1 = sol_b_valid ? q1b : std::nan("");
    q2 = sol_b_valid ? q2b : std::nan("");

    // Throw exception if there is no solution with the
    // HEAD ROT X and HEAD ROT Y within the limits
    if (!sol_b_valid)
      throw std::runtime_error("No solution for the HEAD ROT X and HEAD ROT Y within the limits.");
  }
}


void Human28DOF::ik(const keypoints& measures_in_ext,
                    const std::vector<JointLimits>& qbounds,
                    const Eigen::VectorXd& configuration_previous,
                    Eigen::VectorXd& configuration,
                    Eigen::VectorXd& param)
{
  // 7 dof for chest (tra+quat)
  // 1 dof: shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
  // 1 dof for trunk rotation (around chest_z)
  // 1 dof: hip rotation is the rotation around chest_x_in_ext (frontal direction)
  // 3 dof translation= shoulder_distance, chest_hip_distance, hip_distance
  // 6 dof for each limb: 3 dof shoulder, 1 dof length of the upper arm, 1 dof elbow rotation, 1 dof lenght of the lower arm
  
  // ### TRUNK ###
  Eigen::VectorXd q_trunk(7+3);

  Eigen::VectorXd trunk_param(3);

  std::vector<JointLimits> q_trunk_bounds = {
    qbounds[7],
    qbounds[8],
    qbounds[9]
  };

  Eigen::Affine3d T_ext_rshoulder;
  Eigen::Affine3d T_ext_lshoulder;
  Eigen::Affine3d T_ext_rhip;
  Eigen::Affine3d T_ext_lhip;
  Eigen::Affine3d T_ext_chest;
  trunkIk(measures_in_ext,q_trunk_bounds,q_trunk,trunk_param);
  trunkFk(q_trunk,
          trunk_param,
          T_ext_rshoulder,
          T_ext_lshoulder,
          T_ext_rhip,
          T_ext_lhip,
          T_ext_chest);
  // ### END TRUNK ###

  // ### HEAD ###
  Eigen::VectorXd q_head(2);
  Eigen::VectorXd head_param(1);

  std::vector<JointLimits> q_head_bounds = {
    qbounds[26],
    qbounds[27]
  };

  headIk(measures_in_ext,T_ext_chest,q_head_bounds,q_head,head_param);
  // ### END HEAD ###

  // ### ARMS ###
  Eigen::Vector3d relbow_in_rshoulder=T_ext_rshoulder.inverse()*measures_in_ext.right_elbow;
  Eigen::Vector3d rwrist_in_rshoulder=T_ext_rshoulder.inverse()*measures_in_ext.right_wrist;

  Eigen::Vector3d lelbow_in_lshoulder=T_ext_lshoulder.inverse()*measures_in_ext.left_elbow;
  Eigen::Vector3d lwrist_in_lshoulder=T_ext_lshoulder.inverse()*measures_in_ext.left_wrist;
  
  double upper_arm_length=0.5*(
        (measures_in_ext.right_elbow - measures_in_ext.right_shoulder).norm()+
        (measures_in_ext.left_elbow  - measures_in_ext.left_shoulder).norm());

  double lower_arm_length=0.5*(
        (measures_in_ext.right_elbow - measures_in_ext.right_wrist).norm()+
        (measures_in_ext.left_elbow  - measures_in_ext.left_wrist).norm());

  Eigen::VectorXd arm_param(2);
  arm_param(0)=upper_arm_length;
  arm_param(1)=lower_arm_length;

  std::vector<JointLimits> q_right_arm_bounds = {
    qbounds[10],
    qbounds[11],
    qbounds[12],
    qbounds[13]
  };
  std::vector<JointLimits> q_left_arm_bounds = {
    qbounds[14],
    qbounds[15],
    qbounds[16],
    qbounds[17]
  };

  Eigen::VectorXd q_right_arm_previous(4);
  Eigen::VectorXd q_left_arm_previous(4);
  q_right_arm_previous << configuration_previous.block(10,0,4,1);
  q_left_arm_previous << configuration_previous.block(14,0,4,1);

  Eigen::VectorXd q_right_arm(4);
  Eigen::VectorXd q_left_arm(4);

  // std::cout << "right arm IK" << std::endl;
  rightLimbIk(relbow_in_rshoulder,
              rwrist_in_rshoulder,
              arm_param,
              q_right_arm_bounds,
              q_right_arm_previous,
              q_right_arm);

  // std::cout << "left arm IK" << std::endl;
  leftLimbIk(lelbow_in_lshoulder,
             lwrist_in_lshoulder,
             arm_param,
             q_left_arm_bounds,
              q_left_arm_previous,
             q_left_arm);
  // ### END ARMS ###

  // ### LEGS ###
  Eigen::Vector3d relbow_in_rhip=T_ext_rhip.inverse()*measures_in_ext.right_knee;
  Eigen::Vector3d rwrist_in_rhip=T_ext_rhip.inverse()*measures_in_ext.right_ankle;

  Eigen::Vector3d lelbow_in_lhip=T_ext_lhip.inverse()*measures_in_ext.left_knee;
  Eigen::Vector3d lwrist_in_lhip=T_ext_lhip.inverse()*measures_in_ext.left_ankle;
  
  double upper_leg_length=0.5*(
        (measures_in_ext.right_knee - measures_in_ext.right_hip).norm()+
        (measures_in_ext.left_knee  - measures_in_ext.left_hip).norm());

  double lower_leg_length=0.5*(
        (measures_in_ext.right_knee - measures_in_ext.right_ankle).norm()+
        (measures_in_ext.left_knee  - measures_in_ext.left_ankle).norm());

  Eigen::VectorXd leg_param(2);
  leg_param(0)=upper_leg_length;
  leg_param(1)=lower_leg_length;

  std::vector<JointLimits> q_right_leg_bounds = {
    qbounds[18],
    qbounds[19],
    qbounds[20],
    qbounds[21]
  };
  std::vector<JointLimits> q_left_leg_bounds = {
    qbounds[22],
    qbounds[23],
    qbounds[24],
    qbounds[25]
  };

  Eigen::VectorXd q_right_leg_previous(4);
  Eigen::VectorXd q_left_leg_previous(4);
  q_right_leg_previous << configuration_previous.block(18,0,4,1);
  q_left_leg_previous << configuration_previous.block(22,0,4,1);

  Eigen::VectorXd q_right_leg(4);
  Eigen::VectorXd q_left_leg(4);

  // std::cout << "right leg IK" << std::endl;
  rightLimbIk(relbow_in_rhip,
              rwrist_in_rhip,
              leg_param,
              q_right_leg_bounds,
              q_right_leg_previous,
              q_right_leg);

  // std::cout << "left leg IK" << std::endl;
  leftLimbIk(lelbow_in_lhip,
             lwrist_in_lhip,
             leg_param,
             q_left_leg_bounds,
              q_left_leg_previous,
             q_left_leg);
  // ### END LEGS ###

  configuration.resize(7+3+4*4+2);
  configuration.block(0,0,10,1)= q_trunk;
  configuration.block(10,0,4,1)= q_right_arm;
  configuration.block(14,0,4,1)= q_left_arm;
  configuration.block(18,0,4,1)= q_right_leg;
  configuration.block(22,0,4,1)= q_left_leg;
  configuration.block(26,0,2,1)= q_head;

  param.resize(3+2*2+1);
  param.block(0,0,3,1)=trunk_param;
  param.block(3,0,2,1)=arm_param;
  param.block(5,0,2,1)=leg_param;
  param.block(7,0,1,1)=head_param;
}


std::pair<Eigen::VectorXd, Eigen::VectorXd> Human28DOF::ik_binding(const keypoints& measures_in_ext,
                                                                   const std::vector<JointLimits>& joint_limits,
                                                                   const Eigen::VectorXd& configuration_previous,
                                                                   Eigen::VectorXd& configuration,
                                                                   Eigen::VectorXd& param)
{
  Human28DOF::ik(measures_in_ext,joint_limits,configuration_previous,configuration,param);
  return std::make_pair(configuration,param);
}


void Human28DOF::fk(const Eigen::VectorXd& configuration,
                    const Eigen::VectorXd& param,
                    keypoints& kp_in_ext)
{
  Eigen::VectorXd q_trunk      = configuration.block(0,0,10,1) ;
  Eigen::VectorXd q_right_arm  = configuration.block(10,0,4,1);
  Eigen::VectorXd q_left_arm   = configuration.block(14,0,4,1);
  Eigen::VectorXd q_right_leg  = configuration.block(18,0,4,1);
  Eigen::VectorXd q_left_leg   = configuration.block(22,0,4,1);
  Eigen::VectorXd q_head       = configuration.block(26,0,2,1);

  Eigen::VectorXd trunk_param  = param.block(0,0,3,1);
  Eigen::VectorXd arm_param    = param.block(3,0,2,1);
  Eigen::VectorXd leg_param    = param.block(5,0,2,1);
  Eigen::VectorXd head_param   = param.block(7,0,1,1);


  Eigen::Affine3d T_ext_rshoulder;
  Eigen::Affine3d T_ext_lshoulder;
  Eigen::Affine3d T_ext_rhip;
  Eigen::Affine3d T_ext_lhip;
  Eigen::Affine3d T_ext_chest;
  Eigen::Affine3d T_ext_head;
  trunkFk(q_trunk,
          trunk_param,
          T_ext_rshoulder,
          T_ext_lshoulder,
          T_ext_rhip,
          T_ext_lhip,
          T_ext_chest);
  headFk(q_head,head_param,T_ext_chest,T_ext_head);

  kp_in_ext.head = T_ext_head.translation();
  kp_in_ext.right_shoulder=T_ext_rshoulder.translation();
  kp_in_ext.left_shoulder =T_ext_lshoulder.translation();
  kp_in_ext.right_hip=T_ext_rhip.translation();
  kp_in_ext.left_hip =T_ext_lhip.translation();


  Eigen::Vector3d relbow_in_rshoulder;
  Eigen::Vector3d rwrist_in_rshoulder;

  Eigen::Vector3d lelbow_in_lshoulder;
  Eigen::Vector3d lwrist_in_lshoulder;

  rightLimbFk(q_right_arm,
              arm_param,
              relbow_in_rshoulder,
              rwrist_in_rshoulder);
  leftLimbFk(q_left_arm,
             arm_param,
             lelbow_in_lshoulder,
             lwrist_in_lshoulder);

  Eigen::Vector3d relbow_in_rhip;
  Eigen::Vector3d rwrist_in_rhip;

  Eigen::Vector3d lelbow_in_lhip;
  Eigen::Vector3d lwrist_in_lhip;

  rightLimbFk(q_right_leg,
              leg_param,
              relbow_in_rhip,
              rwrist_in_rhip);
  leftLimbFk(q_left_leg,
             leg_param,
             lelbow_in_lhip,
             lwrist_in_lhip);

  kp_in_ext.right_elbow =T_ext_rshoulder*relbow_in_rshoulder;
  kp_in_ext.right_wrist =T_ext_rshoulder*rwrist_in_rshoulder;
  kp_in_ext.left_elbow  =T_ext_lshoulder*lelbow_in_lshoulder;
  kp_in_ext.left_wrist  =T_ext_lshoulder*lwrist_in_lshoulder;
  kp_in_ext.right_knee  =T_ext_rhip     *relbow_in_rhip     ;
  kp_in_ext.right_ankle =T_ext_rhip     *rwrist_in_rhip     ;
  kp_in_ext.left_knee   =T_ext_lhip     *lelbow_in_lhip     ;
  kp_in_ext.left_ankle  =T_ext_lhip     *lwrist_in_lhip     ;
}


void Human28DOF::fk_tfs(const Eigen::VectorXd& configuration,
                        const Eigen::VectorXd& param,
                        Eigen::Affine3d& T_ext_rshoulder,
                        Eigen::Affine3d& T_ext_lshoulder,
                        Eigen::Affine3d& T_ext_rhip,
                        Eigen::Affine3d& T_ext_lhip,
                        Eigen::Affine3d& T_ext_chest,
                        Eigen::Affine3d& T_ext_head,
                        Eigen::Affine3d& T_ext_rshoulderRotated,
                        Eigen::Affine3d& T_ext_relbow,
                        Eigen::Affine3d& T_ext_rwrist,
                        Eigen::Affine3d& T_ext_lshoulderRotated,
                        Eigen::Affine3d& T_ext_lelbow,
                        Eigen::Affine3d& T_ext_lwrist,
                        Eigen::Affine3d& T_ext_rhipRotated,
                        Eigen::Affine3d& T_ext_rknee,
                        Eigen::Affine3d& T_ext_rankle,
                        Eigen::Affine3d& T_ext_lhipRotated,
                        Eigen::Affine3d& T_ext_lknee,
                        Eigen::Affine3d& T_ext_lankle)
{
  Eigen::VectorXd q_trunk      = configuration.block(0,0,10,1) ;
  Eigen::VectorXd q_right_arm  = configuration.block(10,0,4,1);
  Eigen::VectorXd q_left_arm   = configuration.block(14,0,4,1);
  Eigen::VectorXd q_right_leg  = configuration.block(18,0,4,1);
  Eigen::VectorXd q_left_leg   = configuration.block(22,0,4,1);
  Eigen::VectorXd q_head       = configuration.block(26,0,2,1);

  Eigen::VectorXd trunk_param  = param.block(0,0,3,1);
  Eigen::VectorXd arm_param    = param.block(3,0,2,1);
  Eigen::VectorXd leg_param    = param.block(5,0,2,1);
  Eigen::VectorXd head_param   = param.block(7,0,1,1);

  trunkFk(q_trunk,
          trunk_param,
          T_ext_rshoulder,
          T_ext_lshoulder,
          T_ext_rhip,
          T_ext_lhip,
          T_ext_chest);
  headFk(q_head,head_param,T_ext_chest,T_ext_head);

  Eigen::Affine3d T_rshoulder_rshoulderRotated;
  Eigen::Affine3d T_rshoulder_relbow;
  Eigen::Affine3d T_rshoulder_rwrist;

  Eigen::Affine3d T_lshoulder_lshoulderRotated;
  Eigen::Affine3d T_lshoulder_lelbow;
  Eigen::Affine3d T_lshoulder_lwrist;

  rightLimbFk_tfs(q_right_arm,
                  arm_param,
                  T_rshoulder_rshoulderRotated,
                  T_rshoulder_relbow,
                  T_rshoulder_rwrist);
  leftLimbFk_tfs(q_left_arm,
                 arm_param,
                 T_lshoulder_lshoulderRotated,
                 T_lshoulder_lelbow,
                 T_lshoulder_lwrist);

  T_ext_rshoulderRotated = T_ext_rshoulder*T_rshoulder_rshoulderRotated;
  T_ext_relbow = T_ext_rshoulder*T_rshoulder_relbow;
  T_ext_rwrist = T_ext_rshoulder*T_rshoulder_rwrist;
  T_ext_lshoulderRotated = T_ext_lshoulder*T_lshoulder_lshoulderRotated;
  T_ext_lelbow = T_ext_lshoulder*T_lshoulder_lelbow;
  T_ext_lwrist = T_ext_lshoulder*T_lshoulder_lwrist;

  Eigen::Affine3d T_rhip_rhipRotated;
  Eigen::Affine3d T_rhip_rknee;
  Eigen::Affine3d T_rhip_rankle;

  Eigen::Affine3d T_lhip_lhipRotated;
  Eigen::Affine3d T_lhip_lknee;
  Eigen::Affine3d T_lhip_lankle;

  rightLimbFk_tfs(q_right_leg,
                  leg_param,
                  T_rhip_rhipRotated,
                  T_rhip_rknee,
                  T_rhip_rankle);
  leftLimbFk_tfs(q_left_leg,
                 leg_param,
                 T_lhip_lhipRotated,
                 T_lhip_lknee,
                 T_lhip_lankle);

  T_ext_rhipRotated = T_ext_rhip*T_rhip_rhipRotated;
  T_ext_rknee  = T_ext_rhip*T_rhip_rknee;
  T_ext_rankle = T_ext_rhip*T_rhip_rankle;
  T_ext_lhipRotated = T_ext_lhip*T_lhip_lhipRotated;
  T_ext_lknee  = T_ext_lhip*T_lhip_lknee;
  T_ext_lankle = T_ext_lhip*T_lhip_lankle;
}


double keypoints::keypointDistance(const keypoints& kp1_in_ext,
                                   const keypoints& kp2_in_ext,
                                   keypoints& diff_in_ext)
{
  diff_in_ext.head           = kp1_in_ext.head           - kp2_in_ext.head             ;
  diff_in_ext.left_shoulder  = kp1_in_ext.left_shoulder  - kp2_in_ext.left_shoulder    ;
  diff_in_ext.left_elbow     = kp1_in_ext.left_elbow     - kp2_in_ext.left_elbow       ;
  diff_in_ext.left_wrist     = kp1_in_ext.left_wrist     - kp2_in_ext.left_wrist       ;
  diff_in_ext.left_hip       = kp1_in_ext.left_hip       - kp2_in_ext.left_hip         ;
  diff_in_ext.left_knee      = kp1_in_ext.left_knee      - kp2_in_ext.left_knee        ;
  diff_in_ext.left_ankle     = kp1_in_ext.left_ankle     - kp2_in_ext.left_ankle       ;
  diff_in_ext.right_shoulder = kp1_in_ext.right_shoulder - kp2_in_ext.right_shoulder   ;
  diff_in_ext.right_elbow    = kp1_in_ext.right_elbow    - kp2_in_ext.right_elbow      ;
  diff_in_ext.right_wrist    = kp1_in_ext.right_wrist    - kp2_in_ext.right_wrist      ;
  diff_in_ext.right_hip      = kp1_in_ext.right_hip      - kp2_in_ext.right_hip        ;
  diff_in_ext.right_knee     = kp1_in_ext.right_knee     - kp2_in_ext.right_knee       ;
  diff_in_ext.right_ankle    = kp1_in_ext.right_ankle    - kp2_in_ext.right_ankle      ;

  double distance=0.0;
  distance+=diff_in_ext.head          .norm();
  distance+=diff_in_ext.left_shoulder .norm();
  distance+=diff_in_ext.left_elbow    .norm();
  distance+=diff_in_ext.left_wrist    .norm();
  distance+=diff_in_ext.left_hip      .norm();
  distance+=diff_in_ext.left_knee     .norm();
  distance+=diff_in_ext.left_ankle    .norm();
  distance+=diff_in_ext.right_shoulder.norm();
  distance+=diff_in_ext.right_elbow   .norm();
  distance+=diff_in_ext.right_wrist   .norm();
  distance+=diff_in_ext.right_hip     .norm();
  distance+=diff_in_ext.right_knee    .norm();
  distance+=diff_in_ext.right_ankle   .norm();
  
  return distance;
}

void keypoints::set_keypoints(const std::map<std::string,
                              Eigen::Vector3d>& keypoints)
{
  head = keypoints.at("head");
  left_shoulder = keypoints.at("left_shoulder");
  left_elbow = keypoints.at("left_elbow");
  left_wrist = keypoints.at("left_wrist");
  left_hip = keypoints.at("left_hip");
  left_knee = keypoints.at("left_knee");
  left_ankle = keypoints.at("left_ankle");
  right_shoulder = keypoints.at("right_shoulder");
  right_elbow = keypoints.at("right_elbow");
  right_wrist = keypoints.at("right_wrist");
  right_hip = keypoints.at("right_hip");
  right_knee = keypoints.at("right_knee");
  right_ankle = keypoints.at("right_ankle");
}


const std::vector<double> keypoints::get_keypoints()
{
  std::vector<double> keypoints;
  auto append_vector = [&keypoints](const Eigen::Vector3d& vec) {
      keypoints.push_back(vec.x());
      keypoints.push_back(vec.y());
      keypoints.push_back(vec.z());
  };

  append_vector(head);
  append_vector(left_shoulder);
  append_vector(left_elbow);
  append_vector(left_wrist);
  append_vector(left_hip);
  append_vector(left_knee);
  append_vector(left_ankle);
  append_vector(right_shoulder);
  append_vector(right_elbow);
  append_vector(right_wrist);
  append_vector(right_hip);
  append_vector(right_knee);
  append_vector(right_ankle);

  return keypoints;
}

const std::string keypoints::toString()
{
  std::ostringstream oss;
  oss << "head            = " << head.transpose() << "\n"
      << "left_shoulder   = " << left_shoulder.transpose() << "\n"
      << "left_elbow      = " << left_elbow.transpose() << "\n"
      << "left_wrist      = " << left_wrist.transpose() << "\n"
      << "left_hip        = " << left_hip.transpose() << "\n"
      << "left_knee       = " << left_knee.transpose() << "\n"
      << "left_ankle      = " << left_ankle.transpose() << "\n"
      << "right_shoulder  = " << right_shoulder.transpose() << "\n"
      << "right_elbow     = " << right_elbow.transpose() << "\n"
      << "right_wrist     = " << right_wrist.transpose() << "\n"
      << "right_hip       = " << right_hip.transpose() << "\n"
      << "right_knee      = " << right_knee.transpose() << "\n"
      << "right_ankle     = " << right_ankle.transpose() << "\n";
  return oss.str();
}


std::ostream& operator<<(std::ostream& os, const keypoints& keypoints)
{
  os << "head            = " << keypoints.head            .transpose() << std::endl;
  os << "left_shoulder   = " << keypoints.left_shoulder   .transpose() << std::endl;
  os << "left_elbow      = " << keypoints.left_elbow      .transpose() << std::endl;
  os << "left_wrist      = " << keypoints.left_wrist      .transpose() << std::endl;
  os << "left_hip        = " << keypoints.left_hip        .transpose() << std::endl;
  os << "left_knee       = " << keypoints.left_knee       .transpose() << std::endl;
  os << "left_ankle      = " << keypoints.left_ankle      .transpose() << std::endl;
  os << "right_shoulder  = " << keypoints.right_shoulder  .transpose() << std::endl;
  os << "right_elbow     = " << keypoints.right_elbow     .transpose() << std::endl;
  os << "right_wrist     = " << keypoints.right_wrist     .transpose() << std::endl;
  os << "right_hip       = " << keypoints.right_hip       .transpose() << std::endl;
  os << "right_knee      = " << keypoints.right_knee      .transpose() << std::endl;
  os << "right_ankle     = " << keypoints.right_ankle     .transpose() << std::endl;
  return os;
}

void Human28DOF::print(const Eigen::VectorXd& q,
                       const Eigen::VectorXd& param)
{
  Eigen::VectorXd q_trunk      = q.block(0,0,10,1) ;
  Eigen::VectorXd q_right_arm  = q.block(10,0,4,1);
  Eigen::VectorXd q_left_arm   = q.block(14,0,4,1);
  Eigen::VectorXd q_right_leg  = q.block(18,0,4,1);
  Eigen::VectorXd q_left_leg   = q.block(22,0,4,1);
  Eigen::VectorXd q_head       = q.block(26,0,2,1);

  std::cout << "chest position. x: " << q_trunk(0) << ", y: " << q_trunk(1) << ", z: " << q_trunk(2) << std::endl;
  std::cout << "chest quaternion. x: " << q_trunk(3) << ", y: " << q_trunk(4) << ", z: " << q_trunk(5) << ", w: " << q_trunk(6) << std::endl << std::endl;
  std::cout << "shoulder rotx: " << q_trunk(7) << std::endl;
  std::cout << "hip rotz: " << q_trunk(8) << std::endl;
  std::cout << "hip rotx: " << q_trunk(9) << std::endl << std::endl;

  Eigen::VectorXd qlimb=q_right_arm;

  std::cout << "right arm:" << std::endl;
  std::cout << "1) rotz: " << qlimb(0) << std::endl;
  std::cout << "2) rotx: " << qlimb(1) << std::endl;
  std::cout << "3) roty: " << qlimb(2) << std::endl;
  std::cout << "4) rotz: " << qlimb(3) << std::endl << std::endl;

  qlimb=q_left_arm;

  std::cout << "left arm:" << std::endl;
  std::cout << "1) rotz: " << qlimb(0) << std::endl;
  std::cout << "2) rotx: " << qlimb(1) << std::endl;
  std::cout << "3) roty: " << qlimb(2) << std::endl;
  std::cout << "4) rotz: " << qlimb(3) << std::endl << std::endl;

  qlimb=q_right_leg;

  std::cout << "right leg:" << std::endl;
  std::cout << "1) rotz: " << qlimb(0) << std::endl;
  std::cout << "2) rotx: " << qlimb(1) << std::endl;
  std::cout << "3) roty: " << qlimb(2) << std::endl;
  std::cout << "4) rotz: " << qlimb(3) << std::endl << std::endl;

  qlimb=q_left_leg;

  std::cout << "left leg:" << std::endl;
  std::cout << "1) rotz: " << qlimb(0) << std::endl;
  std::cout << "2) rotx: " << qlimb(1) << std::endl;
  std::cout << "3) roty: " << qlimb(2) << std::endl;
  std::cout << "4) rotz: " << qlimb(3) << std::endl << std::endl;

  std::cout << "head rotz: " << q_head(0) << std::endl;
  std::cout << "head rotx: " << q_head(1) << std::endl << std::endl;
}

}  // end namespace human_model
