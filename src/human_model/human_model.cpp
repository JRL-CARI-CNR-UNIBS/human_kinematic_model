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

void Human28DOF::rightLimbIk(const Eigen::Vector3d& elbow_in_limb,
                             const Eigen::Vector3d& wrist_in_limb,
                             const Eigen::VectorXd& param,
                             const std::vector<JointLimits>& qarm_bounds,
                             Eigen::VectorXd& qarm)
{
  double& q1=qarm(0);  // shoulder rot z
  double& q2=qarm(1);  // shoulder rot x
  double& q3=qarm(2);  // shoulder rot y
  double& q5=qarm(3);  // elbow rot z

  const double& q4=param(0);  // upper arm length
  const double& q6=param(1);  // lower arm length

  double q3min=qarm_bounds[2].min_; // shoulder rot y lower bound
  double q3max=qarm_bounds[2].max_; // shoulder rot y upper bound

  q1=std::atan2(-elbow_in_limb(0),elbow_in_limb(1));
  if (std::abs(std::sin(q1))>0.5)
    q2=std::atan2(elbow_in_limb(2),-elbow_in_limb(0)/std::sin(q1));
  else
    q2=std::atan2(elbow_in_limb(2),elbow_in_limb(1)/std::cos(q1));

  Eigen::AngleAxisd rot01(q1,Eigen::Vector3d::UnitZ());
  Eigen::AngleAxisd rot12(q2,Eigen::Vector3d::UnitX());

  Eigen::Affine3d T01;
  T01=rot01;

  Eigen::Affine3d T12;
  T12=rot12;

  Eigen::Affine3d T02=T01*T12;

  Eigen::Vector3d wrist_in_2=T02.inverse()*wrist_in_limb;

  double q3a(std::atan2(wrist_in_2(2),-wrist_in_2(0)));
  double q3b(std::atan2(-wrist_in_2(2),wrist_in_2(0)));

  // Select the solution with the shoulder rot y within the limits
  if (q3a>q3min && q3a<q3max)
    q3=q3a;
  else if (q3b>q3min && q3b<q3max)
    q3=q3b;
  else
    throw std::runtime_error("No solution for the shoulder rot y within the limits.");

  double q6sinq5;
  if (std::sin(q3)>0.5)
    q6sinq5=wrist_in_2(2)/std::sin(q3);
  else
    q6sinq5=-wrist_in_2(0)/std::cos(q3);

  double q6cosq5=wrist_in_2(1)-q4;

  q5=std::atan2(q6sinq5,q6cosq5);
}


void Human28DOF::leftLimbIk(const Eigen::Vector3d& elbow_in_limb,
                            const Eigen::Vector3d& wrist_in_limb,
                            const Eigen::VectorXd& param,
                            const std::vector<JointLimits>& qarm_bounds,
                            Eigen::VectorXd& qarm)
{
  Eigen::Vector3d mirror_elbow_in_limb=elbow_in_limb;
  Eigen::Vector3d mirror_wrist_in_limb=wrist_in_limb;
  mirror_elbow_in_limb(2)*=-1.0;
  mirror_wrist_in_limb(2)*=-1.0;
  rightLimbIk(mirror_elbow_in_limb,mirror_wrist_in_limb,param,qarm_bounds,qarm);
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


void Human28DOF::leftLimbFk(const Eigen::VectorXd& qarm,
                            const Eigen::VectorXd& param,
                            Eigen::Vector3d& elbow_in_limb,
                            Eigen::Vector3d& wrist_in_limb)
{
  Human28DOF::rightLimbFk(qarm,param,elbow_in_limb,wrist_in_limb);
  elbow_in_limb(2)*=-1.0;
  wrist_in_limb(2)*=-1.0;
}


void Human28DOF::trunkIk(const keypoints& measures_in_ext,
                         Eigen::VectorXd& q,
                         Eigen::VectorXd& param)
{
  q.resize(7+3);
  param.resize(3);

  double& shoulder_rotx=q(7);
  double& hip_rotz=q(8);
  double& hip_rotx=q(9);
  double& shoulder_distance=param(0);
  double& chest_hip_distance=param(1);
  double& hip_distance=param(2);


  Eigen::Vector3d upper_chest=0.5*(measures_in_ext.left_shoulder+measures_in_ext.right_shoulder);
  Eigen::Vector3d lower_chest=0.5*(measures_in_ext.left_hip+measures_in_ext.right_hip);
  Eigen::Vector3d hip_versor_in_ext=(measures_in_ext.left_hip-measures_in_ext.right_hip).normalized();
  Eigen::Vector3d chest_z_in_ext=(upper_chest-lower_chest).normalized();
  Eigen::Vector3d shoulder_versor_in_ext=(measures_in_ext.left_shoulder-measures_in_ext.right_shoulder).normalized();


  shoulder_distance= (measures_in_ext.left_shoulder-measures_in_ext.right_shoulder).norm();
  chest_hip_distance=(upper_chest-lower_chest).norm();
  hip_distance=(measures_in_ext.left_hip-measures_in_ext.right_hip).norm();

  Eigen::Matrix3d chest_rot;

  Eigen::Vector3d chest_y_in_ext=shoulder_versor_in_ext-shoulder_versor_in_ext.dot(chest_z_in_ext)*chest_z_in_ext;
  chest_y_in_ext.normalize();

  Eigen::Vector3d chest_x_in_ext=chest_y_in_ext.cross(chest_z_in_ext);

  chest_rot.col(0)=chest_x_in_ext; // frontal direction
  chest_rot.col(1)=chest_y_in_ext; // right to left shoulder
  chest_rot.col(2)=chest_z_in_ext; // vertical axis (lower chest to upper chest)

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


  Eigen::Vector3d hip_versor_in_chest=T_ext_chest.linear().inverse()*hip_versor_in_ext;
  hip_rotz=std::atan2(-hip_versor_in_chest(0),hip_versor_in_chest(1));

  double cosq2;
  if (std::abs(std::sin(hip_rotz))>0.5)
    cosq2=-hip_versor_in_chest(0)/std::sin(hip_rotz);
  else
    cosq2=hip_versor_in_chest(1)/std::cos(hip_rotz);

  hip_rotx=std::atan2(hip_versor_in_chest(2),cosq2);
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
                        Eigen::VectorXd& q,
                        Eigen::VectorXd& param)
{
  param.resize(1);
  q.resize(2);
  double& q1=q(0);
  double& q2=q(1);

  Eigen::Vector3d head_in_chest=T_ext_chest.inverse() * measures_in_ext.head;
  param(0)=head_in_chest.norm();

  q1=std::atan2(-head_in_chest(1),head_in_chest(2));
  double dcosq2;
  if (std::abs(std::sin(q1))>0.5)
    dcosq2=-head_in_chest(1)/std::sin(q1);
  else
    dcosq2=head_in_chest(2)/std::cos(q1);

  q2=std::atan2(head_in_chest(0),dcosq2);
}


void Human28DOF::ik(const keypoints& measures_in_ext,
                    const std::vector<JointLimits>& qbounds,
                    Eigen::VectorXd& configuration,
                    Eigen::VectorXd& param)
{
  // 7 dof for chest (tra+quat)
  // 1 dof: shoulder rotation is the rotation around chest_x_in_ext (frontal direction)
  // 1 dof for trunk rotation (around chest_z)
  // 1 dof: hip rotation is the rotation around chest_x_in_ext (frontal direction)
  // 3 dof translation= shoulder_distance, chest_hip_distance, hip_distance
  // 6 dof for each limb: 3 dof shoulder, 1 dof length of the upper arm, 1 dof elbow rotation, 1 dof lenght of the lower arm
  Eigen::VectorXd q_trunk(7+3);

  Eigen::VectorXd trunk_param(3);

  Eigen::Affine3d T_ext_rshoulder;
  Eigen::Affine3d T_ext_lshoulder;
  Eigen::Affine3d T_ext_rhip;
  Eigen::Affine3d T_ext_lhip;
  Eigen::Affine3d T_ext_chest;
  trunkIk(measures_in_ext,q_trunk,trunk_param);
  trunkFk(q_trunk,
          trunk_param,
          T_ext_rshoulder,
          T_ext_lshoulder,
          T_ext_rhip,
          T_ext_lhip,
          T_ext_chest);

  Eigen::VectorXd q_head(2);
  Eigen::VectorXd head_param(1);
  headIk(measures_in_ext,T_ext_chest,q_head,head_param);


  double upper_arm_length=0.5*(
        (measures_in_ext.right_elbow - measures_in_ext.right_shoulder).norm()+
        (measures_in_ext.left_elbow  - measures_in_ext.left_shoulder).norm());

  double lower_arm_length=0.5*(
        (measures_in_ext.right_elbow - measures_in_ext.right_wrist).norm()+
        (measures_in_ext.left_elbow  - measures_in_ext.left_wrist).norm());


  double upper_leg_length=0.5*(
        (measures_in_ext.right_knee - measures_in_ext.right_hip).norm()+
        (measures_in_ext.left_knee  - measures_in_ext.left_hip).norm());

  double lower_leg_length=0.5*(
        (measures_in_ext.right_knee - measures_in_ext.right_ankle).norm()+
        (measures_in_ext.left_knee  - measures_in_ext.left_ankle).norm());

  Eigen::VectorXd arm_param(2);
  arm_param(0)=upper_arm_length;
  arm_param(1)=lower_arm_length;

  std::vector<JointLimits> q_arm_bounds = {
    qbounds[10],
    qbounds[11],
    qbounds[12],
    qbounds[13]
  };

  Eigen::VectorXd leg_param(2);
  leg_param(0)=upper_leg_length;
  leg_param(1)=lower_leg_length;

  std::vector<JointLimits> q_leg_bounds = {
    qbounds[18],
    qbounds[19],
    qbounds[20],
    qbounds[21]
  };


  Eigen::Vector3d relbow_in_rshoulder=T_ext_rshoulder.inverse()*measures_in_ext.right_elbow;
  Eigen::Vector3d rwrist_in_rshoulder=T_ext_rshoulder.inverse()*measures_in_ext.right_wrist;

  Eigen::Vector3d lelbow_in_lshoulder=T_ext_lshoulder.inverse()*measures_in_ext.left_elbow;
  Eigen::Vector3d lwrist_in_lshoulder=T_ext_lshoulder.inverse()*measures_in_ext.left_wrist;


  Eigen::VectorXd q_right_arm(4);
  Eigen::VectorXd q_left_arm(4);

  rightLimbIk(relbow_in_rshoulder,
              rwrist_in_rshoulder,
              arm_param,
              q_arm_bounds,
              q_right_arm);
  leftLimbIk(lelbow_in_lshoulder,
             lwrist_in_lshoulder,
             arm_param,
             q_arm_bounds,
             q_left_arm);

  Eigen::Vector3d relbow_in_rhip=T_ext_rhip.inverse()*measures_in_ext.right_knee;
  Eigen::Vector3d rwrist_in_rhip=T_ext_rhip.inverse()*measures_in_ext.right_ankle;

  Eigen::Vector3d lelbow_in_lhip=T_ext_lhip.inverse()*measures_in_ext.left_knee;
  Eigen::Vector3d lwrist_in_lhip=T_ext_lhip.inverse()*measures_in_ext.left_ankle;

  Eigen::VectorXd q_right_leg(4);
  Eigen::VectorXd q_left_leg(4);

  rightLimbIk(relbow_in_rhip,
              rwrist_in_rhip,
              leg_param,
              q_leg_bounds,
              q_right_leg);
  leftLimbIk(lelbow_in_lhip,
             lwrist_in_lhip,
             leg_param,
             q_leg_bounds,
             q_left_leg);


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
                                                                   Eigen::VectorXd& configuration,
                                                                   Eigen::VectorXd& param)
{
  Human28DOF::ik(measures_in_ext,joint_limits,configuration,param);
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
                        Eigen::Affine3d& T_ext_relbow,
                        Eigen::Affine3d& T_ext_rwrist,
                        Eigen::Affine3d& T_ext_lelbow,
                        Eigen::Affine3d& T_ext_lwrist,
                        Eigen::Affine3d& T_ext_rknee,
                        Eigen::Affine3d& T_ext_rankle,
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

  T_ext_relbow = T_ext_rshoulder*Eigen::Translation3d(relbow_in_rshoulder);
  T_ext_rwrist = T_ext_rshoulder*Eigen::Translation3d(rwrist_in_rshoulder);
  T_ext_lelbow = T_ext_lshoulder*Eigen::Translation3d(lelbow_in_lshoulder);
  T_ext_lwrist = T_ext_lshoulder*Eigen::Translation3d(lwrist_in_lshoulder);

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

  T_ext_rknee = T_ext_rhip*Eigen::Translation3d(relbow_in_rhip);
  T_ext_rankle = T_ext_rhip*Eigen::Translation3d(rwrist_in_rhip);
  T_ext_lknee = T_ext_lhip*Eigen::Translation3d(lelbow_in_lhip);
  T_ext_lankle = T_ext_lhip*Eigen::Translation3d(lwrist_in_lhip);
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
