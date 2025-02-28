#include <pybind11/pybind11.h>
#include <pybind11/eigen.h> // to handle the conversion between Eigen matrices and NumPy arrays
#include <pybind11/stl.h> // to handle the conversion between Python dictionaries and C++ maps
#include <human_model/human_model.hpp>

namespace py = pybind11;
using namespace human_model;

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
        .def_static("keypoint_distance", &keypoints::keypointDistance)
        .def("set_keypoints", &keypoints::set_keypoints)
        .def("get_keypoints", &keypoints::get_keypoints)
        .def("to_string", &keypoints::toString);

    py::class_<JointLimits>(m, "JointLimits")
        .def(py::init<double, double>())
        .def_readwrite("min", &JointLimits::min_)
        .def_readwrite("max", &JointLimits::max_);

    py::class_<Human28DOF>(m, "Human28DOF")
        .def(py::init<>())
        .def_static("inverse_kinematics", &Human28DOF::ik_binding)
        .def_static("forward_kinematics", &Human28DOF::fk)
        .def_static("trunkIk", &Human28DOF::trunkIk)
        .def_static("trunkFk", &Human28DOF::trunkFk)
        .def_static("headFk", &Human28DOF::headFk)
        .def_static("headIk", &Human28DOF::headIk)
        .def_static("rightLimbFk", &Human28DOF::rightLimbFk)
        .def_static("leftLimbFk", &Human28DOF::leftLimbFk)
        .def_static("rightLimbIk", &Human28DOF::rightLimbIk)
        .def_static("leftLimbIk", &Human28DOF::leftLimbIk)
        .def_static("default_joint_limits", &Human28DOF::setDefaultJointLimits_binding)
        .def_static("print", &Human28DOF::print);
}