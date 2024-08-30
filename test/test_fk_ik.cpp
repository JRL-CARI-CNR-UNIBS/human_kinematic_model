#include <human_model/human_model.hpp>

int main(int argc, char *argv[])
{

  double shoulder_distance  =0.3;
  double chest_hip_distance =0.4;
  double hip_distance       =0.25;

  double upper_arm_length = 0.3;
  double lower_arm_length = 0.3;
  double upper_leg_length = 0.35;
  double lower_leg_length = 0.4;
  double head_distance    = 0.4;


  Eigen::VectorXd q(7+3+4*4+2);
  Eigen::VectorXd param(3+2+2+1);

  for (size_t idx=0; idx<1e4 ;idx++)
  {
    q.setRandom();
    q.block(3,0,4,1)/=q.block(3,0,4,1).norm();


    param(0)=shoulder_distance;
    param(1)=chest_hip_distance;
    param(2)=hip_distance;
    param(3)=upper_arm_length;
    param(4)=lower_arm_length;
    param(5)=upper_leg_length;
    param(6)=lower_leg_length;
    param(7)=head_distance;


    human_model::keypoints kp_in_ext;
    kp_in_ext.left_hip.x()=1;


    human_model::Human28DOF model;

    human_model::Human28DOF::fk(q,param,kp_in_ext);


    Eigen::VectorXd q2(7+3+4*4+2);
    Eigen::VectorXd param2(3+2+2+1);
    human_model::keypoints kp2_in_ext;
    human_model::keypoints diff_in_ext;


    human_model::Human28DOF::ik(kp_in_ext,q2,param2);

    human_model::Human28DOF::fk(q2,param2,kp2_in_ext);
    double distance = human_model::Human28DOF::keypointDistance(kp_in_ext,kp2_in_ext,diff_in_ext);
   std::cout << "q           : \n" << q.transpose() << std::endl;
   std::cout << "q2          : \n" << q2.transpose() << std::endl;
   std::cout << "diff q      : \n" << (q-q2).transpose() << std::endl;
   std::cout << "param       : \n" << param.transpose() << std::endl;
   std::cout << "param2      : \n" << param2.transpose() << std::endl;
   std::cout << "diff param  : \n" << (param-param2).norm() << std::endl;
   std::cout << "kp          : \n" << kp_in_ext << std::endl;
   std::cout << "kp2         : \n" << kp2_in_ext << std::endl;
   std::cout << "kp diff     : \n" << diff_in_ext << std::endl;
   std::cout << "distance    : \n" << distance << std::endl;

//    human_model::Human28DOF::print(q,param);
    assert(("distance is greater than threshold",distance<1e-8));
    assert(("param difference is greater than threshold",(param-param2).norm()<1e-8));
  }
}
