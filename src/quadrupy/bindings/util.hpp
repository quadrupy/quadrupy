#include <iostream>
#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/LowState_.hpp>
#include <unitree/idl/go2/LowCmd_.hpp>
#include <unitree/common/time/time_tool.hpp>
#include <unitree/common/thread/thread.hpp>
#include <unitree/robot/go2/robot_state/robot_state_client.hpp>

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree::robot::go2;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double PosStopF = (2.146E+9f);
constexpr double VelStopF = (16000.0f);

const int NUM_IMU_ACCEL = 3; // linear xyz
const int NUM_FEET = 4; // quadraped
const int NUM_JOINT_MOTORS = 12; // 20 total, but 12 used

class Go2
{
public:

    explicit Go2(){}
    ~Go2(){}

    void Init();
    void InitRobotStateClient();
    int queryServiceStatus(const std::string& serviceName);
    void activateService(const std::string& serviceName,int activate);
    void set_motor_cmd(std::vector<float> q, std::vector<float> dq, std::vector<float> kp, std::vector<float> kd, std::vector<float> tau);
    void set_crc();
    void write();
    std::array<float, NUM_IMU_ACCEL> imu_accel();
    std::array<float, NUM_IMU_ACCEL> imu_ang_vel();
    std::array<int16_t, NUM_FEET> foot_force();
    std::array<float, NUM_JOINT_MOTORS> q();
    std::array<float, NUM_JOINT_MOTORS> dq();
    std::array<float, NUM_JOINT_MOTORS> tau();
private:
    void InitLowCmd();
    void LowStateMessageHandler(const void* messages);
    void LowCmdWrite();
 
private:
    float Kp = 60.0;
    float Kd = 5.0;
    double time_consume = 0;
    int rate_count = 0;
    int sin_count = 0;
    int motiontime = 0;
    float dt = 0.002; // 0.001~0.01
    std::array<float, NUM_JOINT_MOTORS> q_;
    std::array<float, NUM_JOINT_MOTORS> dq_;
    std::array<float, NUM_JOINT_MOTORS> tau_;
    RobotStateClient rsc;

    std::vector<int> order_fix = {3, 4, 5, 0, 1, 2, 9, 10, 11, 6, 7, 8};

    unitree_go::msg::dds_::LowCmd_ low_cmd{};      // default init
    unitree_go::msg::dds_::LowState_ low_state{};  // default init

    /*publisher*/
    ChannelPublisherPtr<unitree_go::msg::dds_::LowCmd_> lowcmd_publisher;
    /*subscriber*/
    ChannelSubscriberPtr<unitree_go::msg::dds_::LowState_> lowstate_subscriber;

    /*LowCmd write thread*/
    ThreadPtr lowCmdWriteThreadPtr;

    float _targetPos_1[12] = {0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                              -0.2, 1.36, -2.65, 0.2, 1.36, -2.65};

    float _targetPos_2[12] = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                              0.0, 0.67, -1.3, 0.0, 0.67, -1.3};

    float _targetPos_3[12] = {-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                              -0.5, 1.36, -2.65, 0.5, 1.36, -2.65};

    float _startPos[12];
    float _duration_1 = 500;   
    float _duration_2 = 500; 
    float _duration_3 = 1000;   
    float _duration_4 = 900;   
    float _percent_1 = 0;    
    float _percent_2 = 0;    
    float _percent_3 = 0;    
    float _percent_4 = 0;    

    bool firstRun = true;
    bool done = false;
};

uint32_t crc32_core(uint32_t* ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

void Go2::Init()
{
    InitLowCmd();

    /*create publisher*/
    lowcmd_publisher.reset(new ChannelPublisher<unitree_go::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
    lowcmd_publisher->InitChannel();

    /*create subscriber*/
    lowstate_subscriber.reset(new ChannelSubscriber<unitree_go::msg::dds_::LowState_>(TOPIC_LOWSTATE));
    lowstate_subscriber->InitChannel(std::bind(&Go2::LowStateMessageHandler, this, std::placeholders::_1), 1);

    // /*loop publishing thread*/
    // lowCmdWriteThreadPtr = CreateRecurrentThreadEx("writebasiccmd", UT_CPU_ID_NONE, 2000, &Go2::LowCmdWrite, this);
}

void Go2::InitLowCmd()
{
    low_cmd.head()[0] = 0xFE;
    low_cmd.head()[1] = 0xEF;
    low_cmd.level_flag() = 0xFF;
    low_cmd.gpio() = 0;

    for(int i=0; i<20; i++)
    {
        low_cmd.motor_cmd()[i].mode() = (0x01);   // motor switch to servo (PMSM) mode
        low_cmd.motor_cmd()[i].q() = (PosStopF);
        low_cmd.motor_cmd()[i].kp() = (0);
        low_cmd.motor_cmd()[i].dq() = (VelStopF);
        low_cmd.motor_cmd()[i].kd() = (0);
        low_cmd.motor_cmd()[i].tau() = (0);
    }
}

void Go2::InitRobotStateClient()
{
    rsc.SetTimeout(10.0f); 
    rsc.Init();
}

int Go2::queryServiceStatus(const std::string& serviceName)
{
    std::vector<ServiceState> serviceStateList;
    int ret,serviceStatus;
    ret = rsc.ServiceList(serviceStateList);
    size_t i, count=serviceStateList.size();
    for (i=0; i<count; i++)
    {
        const ServiceState& serviceState = serviceStateList[i];
        if(serviceState.name == serviceName)
        {
            if(serviceState.status == 0)
            {
                std::cout << "name: " << serviceState.name <<" is activate"<<std::endl;
                serviceStatus = 1;
            }
            else
            {
                std::cout << "name:" << serviceState.name <<" is deactivate"<<std::endl;
                serviceStatus = 0;
            } 
        }    
    }
    return serviceStatus;
    
}

void Go2::activateService(const std::string& serviceName,int activate)
{
    rsc.ServiceSwitch(serviceName, activate);  
}

void Go2::LowStateMessageHandler(const void* message)
{
    low_state = *(unitree_go::msg::dds_::LowState_*)message;
}

void Go2::set_motor_cmd(std::vector<float> q, std::vector<float> dq, std::vector<float> kp, std::vector<float> kd, std::vector<float> tau) {
    for (auto i = 0; i < 12; ++i) {
        int index = order_fix[i];
        low_cmd.motor_cmd()[i].q() = q[index];
        low_cmd.motor_cmd()[i].dq() = dq[index];
        low_cmd.motor_cmd()[i].kp() = kp[index];
        low_cmd.motor_cmd()[i].kd() = kd[index];
        low_cmd.motor_cmd()[i].tau() = tau[index];
    }
}
void Go2::set_crc() {
    low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
}
void Go2::write() {
    lowcmd_publisher->Write(low_cmd);
}
std::array<float, NUM_IMU_ACCEL> Go2::imu_accel() {
    return low_state.imu_state().accelerometer();
}
std::array<float, NUM_IMU_ACCEL> Go2::imu_ang_vel() {
    return low_state.imu_state().gyroscope();
}
std::array<int16_t, NUM_FEET> Go2::foot_force() {
    std::array<int16_t, NUM_FEET> ff;
    ff[0] = low_state.foot_force()[1];
    ff[1] = low_state.foot_force()[0];
    ff[2] = low_state.foot_force()[3];
    ff[3] = low_state.foot_force()[2];
    return ff;
}
std::array<float, NUM_JOINT_MOTORS> Go2::q() {
    for (auto i=0; i<NUM_JOINT_MOTORS; ++i) {
        q_[order_fix[i]] = low_state.motor_state()[i].q();
    }
    return q_;
}
std::array<float, NUM_JOINT_MOTORS> Go2::dq() {
    for (auto i=0; i<NUM_JOINT_MOTORS; ++i) {
        dq_[order_fix[i]] = low_state.motor_state()[i].dq();
    }
    return dq_;
}
std::array<float, NUM_JOINT_MOTORS> Go2::tau() {
    for (auto i=0; i<NUM_JOINT_MOTORS; ++i) {
        tau_[order_fix[i]] = low_state.motor_state()[i].tau_est();
    }
    return tau_;
}
void Go2::LowCmdWrite()
{
    if(_percent_4<1)
    {
        std::cout<<"Read sensor data example: "<<std::endl;
        std::cout<<"Joint 0 pos: "<<low_state.motor_state()[0].q()<<std::endl;
        std::cout<<"Imu accelerometer : "<<"x: "<<low_state.imu_state().accelerometer()[0]<<" y: "<<low_state.imu_state().accelerometer()[1]<<" z: "<<low_state.imu_state().accelerometer()[2]<<std::endl;
        std::cout<<"Foot force "<<low_state.foot_force()[0]<<std::endl;
        std::cout<<std::endl;
    }
    if((_percent_4 == 1) && ( done == false))
    {
        std::cout<<"The example is done! "<<std::endl;
        std::cout<<std::endl;
        done = true;
    }

    motiontime++;
    if(motiontime>=500)
    {
        if(firstRun)
        {
            for(int i = 0; i < 12; i++)
            {
                _startPos[i] = low_state.motor_state()[i].q();
            }
            firstRun = false;
        }

        _percent_1 += (float)1 / _duration_1;
        _percent_1 = _percent_1 > 1 ? 1 : _percent_1;
        if (_percent_1 < 1)
        {
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_1) * _startPos[j] + _percent_1 * _targetPos_1[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        
        }
        if ((_percent_1 == 1)&&(_percent_2 < 1))
        {
            _percent_2 += (float)1 / _duration_2;
            _percent_2 = _percent_2 > 1 ? 1 : _percent_2;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_2) * _targetPos_1[j] + _percent_2 * _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }

        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3<1))
        {
            _percent_3 += (float)1 / _duration_3;
            _percent_3 = _percent_3 > 1 ? 1 : _percent_3;

            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() =  _targetPos_2[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        if ((_percent_1 == 1)&&(_percent_2 == 1)&&(_percent_3==1)&&((_percent_4<=1)))
        {
            _percent_4 += (float)1 / _duration_4;
            _percent_4 = _percent_4 > 1 ? 1 : _percent_4;
            for (int j = 0; j < 12; j++)
            {
                low_cmd.motor_cmd()[j].q() = (1 - _percent_4) * _targetPos_2[j] + _percent_4 * _targetPos_3[j];
                low_cmd.motor_cmd()[j].dq() = 0;
                low_cmd.motor_cmd()[j].kp() = Kp;
                low_cmd.motor_cmd()[j].kd() = Kd;
                low_cmd.motor_cmd()[j].tau() = 0;
            }
        }
        low_cmd.crc() = crc32_core((uint32_t *)&low_cmd, (sizeof(unitree_go::msg::dds_::LowCmd_)>>2)-1);
        lowcmd_publisher->Write(low_cmd);
    }
   
}
