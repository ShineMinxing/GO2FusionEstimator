#ifndef __FUSION_ESTIMATOR_H_
#define __FUSION_ESTIMATOR_H_

/*
点足腿式里程计算法说明
算法作用：
    使用IMU和关节电机数据，反解腿式机器人状态，提供里程计数据。
算法函数：
    1. 外部调用初始化示例：
        #include "fusion_estimator.h"
        auto Robot_Estimation = CreateRobot_Estimation();
    2. 外部调用状态估计示例：
        #include "FE_LowlevelState.h"
        FE_LowlevelState st{};
        FE_Odometer odom = Robot_Estimation.fusion_estimator(st);
    3. 外部调用算法调参和读取反馈示例：
        double status[200] = {0};
        Robot_Estimation.fusion_estimator(status);
        
        参数设置为：        
        IndexInOrOut = 0,
        IndexStatusOK = 1,

        IndexIMUAccEnable = 2,  
        IndexIMUQuaternionEnable = 3,
        IndexIMUGyroEnable = 4,  

        IndexJointsXYZEnable = 5,
        IndexJointsVelocityXYZEnable = 6,
        IndexJointsRPYEnable = 7,

        IndexLoadedWeight = 9,
        IndexLegFootForceThreshold = 10,  
        IndexLegMinStairHeight = 11,

        IndexLegOrientationInitialWeight = 12, 
        IndexLegOrientationTimeWeight = 13,

        当status[IndexInOrOut]
            ==1：修改估计算法参数
            ==2：读取估计算法参数（建议先读一次，再设置参数，避免把enable设置成false）
            ==3：重置估计的位置为[0,0,0.4]
        
        status[IndexInOrOut]在每次成功设置后，数值会发生变化
        
        为保证更快运行速度，初始模式为：
            status[0:13] = [0,0, 0,1,0, 1,0,0, 0,-80,0.08, 0.001,1000]

算法原理：
    1. 基于IMU估计机器狗姿态角和角速度
    2. 基于关节获得各足在身体坐标系中的位置和速度
    3. 根据身体的世界坐标系位置与方向角，计算各足在世界坐标系中的位置和速度
    4. 基于关节计算足点的z方向作用力，当力超过阈值，认为足落地，并记录落足点[x,y,z]
    5. 根据落足点[x,y,z]，足在地面时，反解身体的位置速度
    6. 对于落足点z，离散化为至少0.08m间隔的高度，每次落地时拉向记录过的高度
*/

#include <memory>
#include <vector>
#include <cmath>
#include <cstdio>
#include <iostream>

#include "SensorBase.h"
#include "Sensor_Legs.h"
#include "Sensor_IMU.h"

struct FE_Odometer
{
    double t = 0.0;

    // 位置/速度/加速度（世界系）
    double pos[3] = {0,0,0};
    double vel[3] = {0,0,0};
    double acc[3] = {0,0,0};

    // 姿态/角速度/角加速度（建议用 rpy 表示，单位 rad）
    double rpy[3] = {0,0,0};
    double rpy_rate[3] = {0,0,0};
    double rpy_acc[3] = {0,0,0};
};

struct FE_LowlevelState
{
    double timestamp = 0; 

    // ===== IMU =====

    double imu_acc[3]  = {0,0,0};         // accelerometer xyz
    double imu_gyro[3] = {0,0,0};         // gyroscope xyz
    double imu_quat[4] = {1,0,0,0};       // quaternion wxyz

    // ===== Motors =====
    static constexpr int MOTOR_COUNT = 16;

    double motor_q[MOTOR_COUNT]   = {0};  // joint position
    double motor_dq[MOTOR_COUNT]  = {0};  // joint velocity
    double motor_tau[MOTOR_COUNT] = {0};  // estimated torque
};

enum ConfigIndex {

    IndexInOrOut = 0,
    IndexStatusOK = 1,
    IndexIMUAccEnable = 2,  
    IndexIMUQuaternionEnable = 3,
    IndexIMUGyroEnable = 4,  
    IndexJointsXYZEnable = 5,
    IndexJointsVelocityXYZEnable = 6,
    IndexJointsRPYEnable = 7,
    
    IndexLoadedWeight = 9,

    IndexLegFootForceThreshold = 10,  
    IndexLegMinStairHeight = 11,
    IndexLegOrientationInitialWeight = 12, 
    IndexLegOrientationTimeWeight = 13,
};

class FusionEstimatorCore
{
public:
    FusionEstimatorCore()
    {
        for (int i = 0; i < 2; ++i) {
            auto *ptr = new EstimatorPortN;
            StateSpaceModel_Go2_Initialization(ptr);
            sensors.emplace_back(ptr);
        }

        imu_acc     = std::make_shared<DataFusion::SensorIMUAcc>    (sensors[0]);
        imu_gyro    = std::make_shared<DataFusion::SensorIMUMagGyro>(sensors[1]);
        legs_pos    = std::make_shared<DataFusion::SensorLegsPos>   (sensors[0]);
        legs_ori    = std::make_shared<DataFusion::SensorLegsOri>   (sensors[1]);
        legs_ori->SetLegsPosRef(legs_pos.get());

        yaw_correct = 0.0;
    }

    ~FusionEstimatorCore()
    {
        for (auto* p : sensors) delete p;
        sensors.clear();
    }

    FusionEstimatorCore(const FusionEstimatorCore&) = delete;
    FusionEstimatorCore& operator=(const FusionEstimatorCore&) = delete;

    FusionEstimatorCore(FusionEstimatorCore&&) noexcept = default;
    FusionEstimatorCore& operator=(FusionEstimatorCore&&) noexcept = default;

    void fusion_estimator_status(double status[200])
    {
        if (status[IndexInOrOut] == 1) 
        {
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 1;

            if(!(status[IndexIMUAccEnable]||status[IndexIMUQuaternionEnable]||status[IndexIMUGyroEnable]||status[IndexJointsXYZEnable]||status[IndexJointsRPYEnable]))
                status[IndexStatusOK] = -999;

            imu_acc->IMUAccEnable               = status[IndexIMUAccEnable];
            imu_gyro->IMUQuaternionEnable       = status[IndexIMUQuaternionEnable];
            imu_gyro->IMUGyroEnable             = status[IndexIMUGyroEnable];
            legs_pos->JointsXYZEnable           = status[IndexJointsXYZEnable];
            legs_pos->JointsXYZVelocityEnable   = status[IndexJointsVelocityXYZEnable];
            legs_ori->JointsRPYEnable           = status[IndexJointsRPYEnable];

            legs_pos->LoadedWeight              = status[IndexLoadedWeight];

            legs_pos->FootEffortThreshold       = status[IndexLegFootForceThreshold];
            legs_pos->Environement_Height_Scope = status[IndexLegMinStairHeight];
            legs_ori->legori_init_weight        = status[IndexLegOrientationInitialWeight];
            legs_ori->legori_time_weight        = status[IndexLegOrientationTimeWeight];
        }
        else if (status[IndexInOrOut] == 2){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 10;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;

            status[IndexIMUAccEnable]             = imu_acc->IMUAccEnable;
            status[IndexIMUQuaternionEnable]      = imu_gyro->IMUQuaternionEnable;
            status[IndexIMUGyroEnable]            = imu_gyro->IMUGyroEnable;
            status[IndexJointsXYZEnable]          = legs_pos->JointsXYZEnable;
            status[IndexJointsVelocityXYZEnable]  = legs_pos->JointsXYZVelocityEnable;
            status[IndexJointsRPYEnable]          = legs_ori->JointsRPYEnable;
                
            status[IndexLoadedWeight]             = legs_pos->LoadedWeight;

            status[IndexLegFootForceThreshold]    = legs_pos->FootEffortThreshold;
            status[IndexLegMinStairHeight]        = legs_pos->Environement_Height_Scope;
            status[IndexLegOrientationInitialWeight] = legs_ori->legori_init_weight;
            status[IndexLegOrientationTimeWeight]    = legs_ori->legori_time_weight;

            for(int i = 0; i < 9; i++){
                status[50 + i] = sensors[0]->EstimatedState[i];
            }
            for(int i = 0; i < 9; i++){
                status[60 + i] = sensors[1]->EstimatedState[i];
            }
            
            for(int i = 0; i < 100; i++){
                status[100 + i] = sensors[0]->Double_Par[i];
            }
        }
        else if (status[IndexInOrOut] == 3){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 20;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            sensors[0]->EstimatedState[0] = 0;
            sensors[0]->EstimatedState[3] = 0;
            sensors[0]->EstimatedState[6] = 0;
            yaw_correct = - sensors[1]->EstimatedState[6];
            legs_pos->FootfallPositionRecordIsInitiated[0] = false;
            legs_pos->FootfallPositionRecordIsInitiated[1] = false;
            legs_pos->FootfallPositionRecordIsInitiated[2] = false;
            legs_pos->FootfallPositionRecordIsInitiated[3] = false;
        }
        else if (status[IndexInOrOut] == 4){
            status[IndexInOrOut] = 0;
            status[IndexStatusOK] = status[IndexStatusOK] + 40;
            if (status[IndexStatusOK] > 999)
                status[IndexStatusOK] = 1;
            legs_pos->UseGo2P();

        }
    }

    FE_Odometer fusion_estimator(const FE_LowlevelState& st)
    {
        FE_Odometer odom;

        const double q[4] = {
            static_cast<double>(st.imu_quat[0]),
            static_cast<double>(st.imu_quat[1]),
            static_cast<double>(st.imu_quat[2]),
            static_cast<double>(st.imu_quat[3])
        };

        if(!DataFusion::quat_is_ok(q))
            return odom;

        const double CurrentTimestamp = st.timestamp;
        static double LastUsedTimestamp = 0, StartTimeStamp = 0;

        if (!(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp < 1) || !(CurrentTimestamp - StartTimeStamp - LastUsedTimestamp >0))
            StartTimeStamp = CurrentTimestamp - LastUsedTimestamp;

        double UsedTimestamp = CurrentTimestamp - StartTimeStamp;
        LastUsedTimestamp = UsedTimestamp;

        if (imu_acc->IMUAccEnable) {
            double msg_acc[9] = {0};
            msg_acc[3*0 + 2] = static_cast<double>(st.imu_acc[0]);
            msg_acc[3*1 + 2] = static_cast<double>(st.imu_acc[1]);
            msg_acc[3*2 + 2] = static_cast<double>(st.imu_acc[2]);
            
            if(Signal_Available_Check(msg_acc,0))
                imu_acc->SensorDataHandle(msg_acc, UsedTimestamp);
        }

        double roll, pitch, yaw;
        double msg_rpy[9] = {0};

        DataFusion::quat_to_eulerZYX(q, roll, pitch, yaw);

        msg_rpy[3*0] = roll;
        msg_rpy[3*1] = pitch;
        msg_rpy[3*2] = yaw + yaw_correct;

        msg_rpy[3*0 + 1] = static_cast<double>(st.imu_gyro[0]);
        msg_rpy[3*1 + 1] = static_cast<double>(st.imu_gyro[1]);
        msg_rpy[3*2 + 1] = static_cast<double>(st.imu_gyro[2]);

        if(Signal_Available_Check(msg_rpy,1))
            imu_gyro->SensorDataHandle(msg_rpy, UsedTimestamp);
        
        if (legs_pos->JointsXYZEnable||legs_pos->JointsXYZVelocityEnable){
            double joint[48];

            for (int i = 0; i < 16; ++i) {
                joint[0 + i]  = st.motor_q[i];
                joint[16 + i] = st.motor_dq[i];
                joint[32 + i] = st.motor_tau[i];
            }

            if (Signal_Available_Check(joint,2)||legs_pos->CalculateWeightEnable) {
                legs_pos->SensorDataHandle(joint, UsedTimestamp);
                
                if (legs_ori->JointsRPYEnable) {
                    const double last_yaw = sensors[1]->EstimatedState[6] - yaw_correct;

                    legs_ori->SensorDataHandle(joint, UsedTimestamp);

                    yaw_correct = legs_ori->legori_correct - last_yaw;

                    if (!imu_gyro->IMUQuaternionEnable) {
                        sensors[1]->EstimatedState[6] = legs_ori->legori_correct;
                    }
                }
            }
        }

        // ===== 位置/速度/加速度：来自 sensors[0] =====
        odom.pos[0] = static_cast<float>(sensors[0]->EstimatedState[0]);
        odom.pos[1] = static_cast<float>(sensors[0]->EstimatedState[3]);
        odom.pos[2] = static_cast<float>(sensors[0]->EstimatedState[6]);
        odom.vel[0] = static_cast<float>(sensors[0]->EstimatedState[1]);
        odom.vel[1] = static_cast<float>(sensors[0]->EstimatedState[4]);
        odom.vel[2] = static_cast<float>(sensors[0]->EstimatedState[7]);
        odom.acc[0] = static_cast<float>(sensors[0]->EstimatedState[2]);
        odom.acc[1] = static_cast<float>(sensors[0]->EstimatedState[5]);
        odom.acc[2] = static_cast<float>(sensors[0]->EstimatedState[8]);

        // ===== 姿态角/角速度/角加速度：来自 sensors[1] =====
        odom.rpy[0]  = static_cast<float>(sensors[1]->EstimatedState[0]);
        odom.rpy[1] = static_cast<float>(sensors[1]->EstimatedState[3]);
        odom.rpy[2]   = static_cast<float>(sensors[1]->EstimatedState[6]);
        odom.rpy_rate[0]  = static_cast<float>(sensors[1]->EstimatedState[1]);
        odom.rpy_rate[1] = static_cast<float>(sensors[1]->EstimatedState[4]);
        odom.rpy_rate[2]   = static_cast<float>(sensors[1]->EstimatedState[7]);
        odom.rpy_acc[0]  = static_cast<float>(legs_pos->FootfallPar[0]);
        odom.rpy_acc[1] = static_cast<float>(legs_pos->FootfallPar[1]);
        odom.rpy_acc[2]   = static_cast<float>(legs_pos->FootfallPar[2]);
        
        return odom;
    }

private:
    std::vector<EstimatorPortN*> sensors;

    std::shared_ptr<DataFusion::SensorIMUAcc>     imu_acc;
    std::shared_ptr<DataFusion::SensorIMUMagGyro> imu_gyro;
    std::shared_ptr<DataFusion::SensorLegsPos>    legs_pos;
    std::shared_ptr<DataFusion::SensorLegsOri>    legs_ori;

    double legori_init_weight, legori_time_weight;
    double yaw_correct;

    bool Signal_Available_Check(double Signal[], int type)
    {
        static double last[3][48] = {0};
        static int Number[3] = {9,9,48};
        bool diff = false;

        for (int i = 0; i < Number[type]; ++i) {
            if (!(Signal[i] < 9999.0 && Signal[i] > -9999.0))
                return false;
            if (Signal[i] != last[type][i])
                diff = true;
        }
        if(!diff)
            return false;
        else
            for (int i = 0; i < Number[type]; ++i)
                last[type][i] = Signal[i];
        return true;
    }
};

inline FusionEstimatorCore CreateRobot_Estimation()
{
    return FusionEstimatorCore{};
}

#endif  /* __FUSION_ESTIMATOR_H_ */