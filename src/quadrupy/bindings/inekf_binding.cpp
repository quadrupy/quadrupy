#include "InEKF.h"
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <array>
#include <math.h>
#include <pybind11/operators.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <vector>

using namespace inekf;

namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::map<int, Eigen::Vector3d>);

PYBIND11_MODULE(inekf_py, m) {
    py::class_<RobotState>(m, "RobotState")
        .def(py::init<>())
        .def("setRotation",&RobotState::setRotation)
        .def("setVelocity",&RobotState::setVelocity)
        .def("setPosition",&RobotState::setPosition)
        .def("setGyroscopeBias",&RobotState::setGyroscopeBias)
        .def("setAccelerometerBias",&RobotState::setAccelerometerBias)
        .def("getRotation",&RobotState::getRotation)
        .def("getVelocity",&RobotState::getVelocity)
        .def("getPosition",&RobotState::getPosition)
        .def("getGyroscopeBias",&RobotState::getGyroscopeBias)
        .def("getAccelerometerBias",&RobotState::getAccelerometerBias);

    py::class_<NoiseParams>(m, "NoiseParams")
        .def(py::init<>())
        .def("setGyroscopeNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setGyroscopeNoise), "set the gryroscope noise")
        .def("setAccelerometerNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setAccelerometerNoise), "set the accelerometer noise")
        .def("setGyroscopeBiasNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setGyroscopeBiasNoise), "set the gryroscope bias noise")
        .def("setAccelerometerBiasNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setAccelerometerNoise), "set the accelerometer bias noise")
        .def("setContactNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setContactNoise), "set the contact noise")
        .def("setContactNoise",static_cast<void (NoiseParams::*)(const Eigen::Vector3d&)>(&NoiseParams::setContactNoise), "set the contact noise")
        .def("setLandmarkNoise",static_cast<void (NoiseParams::*)(double_t)>(&NoiseParams::setLandmarkNoise), "set the landmark noise")
        .def("setLandmarkNoise",static_cast<void (NoiseParams::*)(const Eigen::Vector3d&)>(&NoiseParams::setLandmarkNoise), "set the landmark noise")
        .def("getGyroscopeCov",&NoiseParams::getGyroscopeCov)
        .def("getGyroscopeBiasCov",&NoiseParams::getGyroscopeBiasCov)
        .def("getAccelerometerBiasCov",&NoiseParams::getAccelerometerBiasCov)
        .def("getLandmarkCov",&NoiseParams::getLandmarkCov)
        .def("getContactCov",&NoiseParams::getContactCov);

    py::class_<Kinematics>(m, "Kinematics")
        .def(py::init<int, Eigen::Matrix4d, Eigen::Matrix<double,6,6>>())
        .def_readwrite("pose", &Kinematics::pose)
        .def_readwrite("covariance", &Kinematics::covariance);
    
    py::class_<Landmark>(m, "Landmark")
        .def(py::init<int, Eigen::Vector3d>())
        .def_readwrite("position", &Landmark::position);

    py::class_<InEKF>(m, "InEKF")
        .def(py::init<>())
        .def(py::init<NoiseParams>())
        .def(py::init<RobotState>())
        .def(py::init<RobotState,NoiseParams>())
        .def("getState",&InEKF::getState)
        .def("getEstimatedContactPositions",&InEKF::getEstimatedContactPositions)
        .def("setState",&InEKF::setState)
        .def("setContacts",&InEKF::setContacts)
        .def("setPriorLandmarks",&InEKF::setPriorLandmarks)
        .def("Propagate",&InEKF::Propagate)
        .def("Correct",&InEKF::Correct)
        .def("CorrectKinematics",&InEKF::CorrectKinematics)
        .def("CorrectLandmarks",&InEKF::CorrectLandmarks);
}
