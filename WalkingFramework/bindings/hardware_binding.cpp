#include "unitree/idl/go2/LowCmd_.hpp"
#include "unitree/idl/go2/MotorCmd_.hpp"
#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/go2/robot_state/robot_state_client.hpp"
#include "util.hpp"
#include <iostream>
#include <unistd.h>
#include <string.h>
#include <array>
#include <math.h>
#include <pybind11/pybind11.h>
#include <pybind11/operators.h>
#include <pybind11/stl.h>
#include <pybind11/stl_bind.h>
#include <pybind11/eigen.h>
#include <vector>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

using unitree_go::msg::dds_::LowCmd_;
using unitree_go::msg::dds_::MotorCmd_;



namespace py = pybind11;

PYBIND11_MAKE_OPAQUE(std::map<int, Eigen::Vector3d>);

PYBIND11_MODULE(go2_py, m) {
    // py::class_<LowCmd_>(m, "LowCmd_")
    //     .def(py::init<>())
    //     .def("head", &LowCmd_::head)
    //     .def("level_flag",&LowCmd_::level_flag)
    //     .def("gpio",&LowCmd_::gpio)
    //     .def("motor_cmd",&LowCmd_::motor_cmd);
    //     // .def("crc",&LowCmd_::crc);
    // py::class_<MotorCmd_>(m, "MotorCmd")
    //     .def(py::init<>())
    //     .def("mode", &MotorCmd_::mode)
    //     .def_property("q", &MotorCmd_::q, [](MotorCmd_& mc, float q) {mc.q() = q;})
    //     .def_property("dq", &MotorCmd_::dq, [](MotorCmd_& mc, float dq) {mc.dq() = dq;})
    //     .def_property("tau", &MotorCmd_::tau, [](MotorCmd_& mc, float tau) {mc.tau() = tau;})
    //     .def_property("kp", &MotorCmd_::kp, [](MotorCmd_& mc, float kp) {mc.kp() = kp;})
    //     .def_property("kd", &MotorCmd_::kd, [](MotorCmd_& mc, float kd) {mc.kd() = kd;});
    py::class_<ChannelFactory>(m, "ChannelFactory")
        .def("InstanceInit", [](std::string interface){ChannelFactory::Instance()->Init(0, interface);});

    // py::class_<RobotStateClient>(m, "RobotStateClient")
    //     .def(py::init<>())
    //     .def("init", &RobotStateClient::Init)
    //     .def("set_timeout", &RobotStateClient::SetTimeout);
    
    py::class_<Go2>(m, "Go2")
        .def(py::init<>())
        .def("init", &Go2::Init)
        .def("init_robot_state_client", &Go2::InitRobotStateClient)
        .def("query_service_status", &Go2::queryServiceStatus)
        .def("activate_service", &Go2::activateService)
        .def("set_motor_cmd", &Go2::set_motor_cmd)
        .def("set_crc", &Go2::set_crc)
        .def("write", &Go2::write)
        .def("imu_accel", &Go2::imu_accel)
        .def("foot_force", &Go2::foot_force)
        .def("q", &Go2::q)
        .def("dq", &Go2::dq)
        .def("tau", &Go2::tau);
}
