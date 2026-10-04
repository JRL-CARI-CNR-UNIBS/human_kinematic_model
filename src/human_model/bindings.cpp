#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // to handle the conversion between Eigen matrices and NumPy arrays
#include <pybind11/stl.h> // to handle the conversion between Python dictionaries and C++ maps
#include <human_model/human_model.hpp>

namespace py = pybind11;
using namespace human_model;

// pybind11 copies Eigen arguments passed by non-const reference, so the C++ output arguments are
// never seen from Python: every function is wrapped to return its outputs instead.
// Eigen::Affine3d has no pybind11 type caster: transforms are exchanged as 4x4 numpy arrays.

static Eigen::Affine3d toAffine(const Eigen::Matrix4d& matrix)
{
  Eigen::Affine3d T;
  T.matrix() = matrix;
  return T;
}

PYBIND11_MODULE(human_model_binding, m) {
    py::class_<keypoints>(m, "Keypoints")
        .def(py::init<>())
        .def_readwrite("head", &keypoints::head)
        .def_readwrite("left_shoulder", &keypoints::left_shoulder)
        .def_readwrite("left_elbow", &keypoints::left_elbow)
        .def_readwrite("left_wrist", &keypoints::left_wrist)
        .def_readwrite("left_hip", &keypoints::left_hip)
        .def_readwrite("left_knee", &keypoints::left_knee)
        .def_readwrite("left_ankle", &keypoints::left_ankle)
        .def_readwrite("right_shoulder", &keypoints::right_shoulder)
        .def_readwrite("right_elbow", &keypoints::right_elbow)
        .def_readwrite("right_wrist", &keypoints::right_wrist)
        .def_readwrite("right_hip", &keypoints::right_hip)
        .def_readwrite("right_knee", &keypoints::right_knee)
        .def_readwrite("right_ankle", &keypoints::right_ankle)
        // keypoint_distance(kp1, kp2, diff) -> distance, with diff filled in place
        .def_static("keypoint_distance", &keypoints::keypointDistance,
                    py::arg("kp1_in_ext"), py::arg("kp2_in_ext"), py::arg("diff_in_ext"))
        // keypoint_distance(kp1, kp2) -> (distance, diff), as in the python translation
        .def_static("keypoint_distance",
                    [](const keypoints& kp1_in_ext, const keypoints& kp2_in_ext) {
                        keypoints diff_in_ext;
                        double distance = keypoints::keypointDistance(kp1_in_ext, kp2_in_ext, diff_in_ext);
                        return std::make_tuple(distance, diff_in_ext);
                    },
                    py::arg("kp1_in_ext"), py::arg("kp2_in_ext"))
        .def("set_keypoints", &keypoints::set_keypoints)
        .def("get_keypoints", &keypoints::get_keypoints)
        .def("to_string", &keypoints::toString);

    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<double, double>())
        .def_readwrite("min", &JointLimits::min_)
        .def_readwrite("max", &JointLimits::max_);

    py::class_<Human28DOF>(m, "Human28DOF")
        .def(py::init<>())

        // inverse_kinematics(kp, limits, q_previous) -> (q, param, chest_q_rotated)
        .def_static("inverse_kinematics",
                    [](const keypoints& measures_in_ext,
                       const std::vector<JointLimits>& joint_limits,
                       const Eigen::VectorXd& configuration_previous) {
                        Eigen::VectorXd configuration, param;
                        Eigen::Vector4d chest_q_rotated;
                        Human28DOF::ik(measures_in_ext, joint_limits, configuration_previous,
                                       configuration, param, chest_q_rotated);
                        return std::make_tuple(configuration, param, chest_q_rotated);
                    },
                    py::arg("measures_in_ext"), py::arg("joint_limits"), py::arg("configuration_previous"))
        // legacy signature: the last three (output) arguments are ignored, the outputs are returned
        .def_static("inverse_kinematics", &Human28DOF::ik_binding,
                    py::arg("measures_in_ext"), py::arg("joint_limits"), py::arg("configuration_previous"),
                    py::arg("configuration"), py::arg("param"), py::arg("chest_q_rotated"))

        // forward_kinematics(q, param) -> Keypoints
        .def_static("forward_kinematics",
                    [](const Eigen::VectorXd& configuration, const Eigen::VectorXd& param) {
                        keypoints kp_in_ext;
                        Human28DOF::fk(configuration, param, kp_in_ext);
                        return kp_in_ext;
                    },
                    py::arg("configuration"), py::arg("param"))
        // legacy signature: forward_kinematics(q, param, kp) fills kp in place
        .def_static("forward_kinematics", &Human28DOF::fk,
                    py::arg("configuration"), py::arg("param"), py::arg("kp_in_ext"))

        // forward_kinematics_tfs(q, param) -> {name: 4x4 transform}, names as in Human28DOF::fk_tfs
        .def_static("forward_kinematics_tfs",
                    [](const Eigen::VectorXd& configuration, const Eigen::VectorXd& param) {
                        std::array<Eigen::Affine3d, 18> T;
                        Human28DOF::fk_tfs(configuration, param,
                                           T[0], T[1], T[2], T[3], T[4], T[5], T[6], T[7], T[8],
                                           T[9], T[10], T[11], T[12], T[13], T[14], T[15], T[16], T[17]);
                        const std::array<const char*, 18> names = {
                            "T_ext_rshoulder", "T_ext_lshoulder", "T_ext_rhip", "T_ext_lhip",
                            "T_ext_chest", "T_ext_head",
                            "T_ext_rshoulderRotated", "T_ext_relbow", "T_ext_rwrist",
                            "T_ext_lshoulderRotated", "T_ext_lelbow", "T_ext_lwrist",
                            "T_ext_rhipRotated", "T_ext_rknee", "T_ext_rankle",
                            "T_ext_lhipRotated", "T_ext_lknee", "T_ext_lankle"};
                        std::map<std::string, Eigen::Matrix4d> tfs;
                        for (size_t i = 0; i < names.size(); i++)
                            tfs[names[i]] = T[i].matrix();
                        return tfs;
                    },
                    py::arg("configuration"), py::arg("param"))

        // trunkIk(kp, qtrunk_bounds) -> (q_trunk, param_trunk, chest_q_rotated)
        .def_static("trunkIk",
                    [](const keypoints& measures_in_ext, const std::vector<JointLimits>& qtrunk_bounds) {
                        Eigen::VectorXd q, param;
                        Eigen::Vector4d chest_q_rotated;
                        Human28DOF::trunkIk(measures_in_ext, qtrunk_bounds, q, param, chest_q_rotated);
                        return std::make_tuple(q, param, chest_q_rotated);
                    },
                    py::arg("measures_in_ext"), py::arg("qtrunk_bounds"))

        // trunkFk(q_trunk, param_trunk) -> (T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest)
        .def_static("trunkFk",
                    [](const Eigen::VectorXd& q, const Eigen::VectorXd& param) {
                        Eigen::Affine3d T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip, T_ext_chest;
                        Human28DOF::trunkFk(q, param, T_ext_rshoulder, T_ext_lshoulder, T_ext_rhip, T_ext_lhip,
                                            T_ext_chest);
                        return std::make_tuple(Eigen::Matrix4d(T_ext_rshoulder.matrix()),
                                               Eigen::Matrix4d(T_ext_lshoulder.matrix()),
                                               Eigen::Matrix4d(T_ext_rhip.matrix()),
                                               Eigen::Matrix4d(T_ext_lhip.matrix()),
                                               Eigen::Matrix4d(T_ext_chest.matrix()));
                    },
                    py::arg("q"), py::arg("param"))

        // headFk(q_head, param_head, T_ext_chest) -> T_ext_head
        .def_static("headFk",
                    [](const Eigen::VectorXd& q, const Eigen::VectorXd& param, const Eigen::Matrix4d& T_ext_chest) {
                        Eigen::Affine3d T_ext_head;
                        Human28DOF::headFk(q, param, toAffine(T_ext_chest), T_ext_head);
                        return Eigen::Matrix4d(T_ext_head.matrix());
                    },
                    py::arg("q"), py::arg("param"), py::arg("T_ext_chest"))

        // headIk(kp, T_ext_chest, qhead_bounds) -> (q_head, param_head)
        .def_static("headIk",
                    [](const keypoints& measures_in_ext, const Eigen::Matrix4d& T_ext_chest,
                       const std::vector<JointLimits>& qhead_bounds) {
                        Eigen::VectorXd q, param;
                        Human28DOF::headIk(measures_in_ext, toAffine(T_ext_chest), qhead_bounds, q, param);
                        return std::make_tuple(q, param);
                    },
                    py::arg("measures_in_ext"), py::arg("T_ext_chest"), py::arg("qhead_bounds"))

        // rightLimbFk / leftLimbFk(qarm, param) -> (elbow_in_limb, wrist_in_limb)
        .def_static("rightLimbFk",
                    [](const Eigen::VectorXd& qarm, const Eigen::VectorXd& param) {
                        Eigen::Vector3d elbow_in_limb, wrist_in_limb;
                        Human28DOF::rightLimbFk(qarm, param, elbow_in_limb, wrist_in_limb);
                        return std::make_tuple(elbow_in_limb, wrist_in_limb);
                    },
                    py::arg("qarm"), py::arg("param"))
        .def_static("leftLimbFk",
                    [](const Eigen::VectorXd& qarm, const Eigen::VectorXd& param) {
                        Eigen::Vector3d elbow_in_limb, wrist_in_limb;
                        Human28DOF::leftLimbFk(qarm, param, elbow_in_limb, wrist_in_limb);
                        return std::make_tuple(elbow_in_limb, wrist_in_limb);
                    },
                    py::arg("qarm"), py::arg("param"))

        // rightLimbFk_tfs / leftLimbFk_tfs(qarm, param) -> (T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist)
        .def_static("rightLimbFk_tfs",
                    [](const Eigen::VectorXd& qarm, const Eigen::VectorXd& param) {
                        Eigen::Affine3d T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist;
                        Human28DOF::rightLimbFk_tfs(qarm, param, T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist);
                        return std::make_tuple(Eigen::Matrix4d(T_limb_shoulderRotated.matrix()),
                                               Eigen::Matrix4d(T_limb_elbow.matrix()),
                                               Eigen::Matrix4d(T_limb_wrist.matrix()));
                    },
                    py::arg("qarm"), py::arg("param"))
        .def_static("leftLimbFk_tfs",
                    [](const Eigen::VectorXd& qarm, const Eigen::VectorXd& param) {
                        Eigen::Affine3d T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist;
                        Human28DOF::leftLimbFk_tfs(qarm, param, T_limb_shoulderRotated, T_limb_elbow, T_limb_wrist);
                        return std::make_tuple(Eigen::Matrix4d(T_limb_shoulderRotated.matrix()),
                                               Eigen::Matrix4d(T_limb_elbow.matrix()),
                                               Eigen::Matrix4d(T_limb_wrist.matrix()));
                    },
                    py::arg("qarm"), py::arg("param"))

        // rightLimbIk / leftLimbIk(elbow_in_limb, wrist_in_limb, param, qarm_bounds, qarm_previous) -> qarm
        .def_static("rightLimbIk",
                    [](const Eigen::Vector3d& elbow_in_limb, const Eigen::Vector3d& wrist_in_limb,
                       const Eigen::VectorXd& param, const std::vector<JointLimits>& qarm_bounds,
                       const Eigen::VectorXd& qarm_previous) {
                        Eigen::VectorXd qarm(4);
                        Human28DOF::rightLimbIk(elbow_in_limb, wrist_in_limb, param, qarm_bounds, qarm_previous, qarm);
                        return qarm;
                    },
                    py::arg("elbow_in_limb"), py::arg("wrist_in_limb"), py::arg("param"),
                    py::arg("qarm_bounds"), py::arg("qarm_previous"))
        .def_static("leftLimbIk",
                    [](const Eigen::Vector3d& elbow_in_limb, const Eigen::Vector3d& wrist_in_limb,
                       const Eigen::VectorXd& param, const std::vector<JointLimits>& qarm_bounds,
                       const Eigen::VectorXd& qarm_previous) {
                        Eigen::VectorXd qarm(4);
                        Human28DOF::leftLimbIk(elbow_in_limb, wrist_in_limb, param, qarm_bounds, qarm_previous, qarm);
                        return qarm;
                    },
                    py::arg("elbow_in_limb"), py::arg("wrist_in_limb"), py::arg("param"),
                    py::arg("qarm_bounds"), py::arg("qarm_previous"))

        .def_static("default_joint_limits", &Human28DOF::setDefaultJointLimits_binding)
        .def_static("print", &Human28DOF::print);
}
