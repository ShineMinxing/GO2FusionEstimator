#include "Sensor_Legs.h"

namespace DataFusion
{
    void SensorLegsPos::SensorDataHandle(double* Message, double Time) 
    {
        if((!JointsXYZEnable)&&(!JointsXYZVelocityEnable)){
            return;
        }

        int i, LegNumber;
        ObservationTime = Time;
        
        for(i = 0; i < StateSpaceModel->Nz; i++)
            Observation[i] = 0;

        for(LegNumber = 0; LegNumber<4; LegNumber++)
        {
            
            for(i = 0; i < 3; i++){
                LatestFootEffort[LegNumber][i] = 0;
            }
            for(i = 0; i < 6; i++){
                FeetEffort2BodyMotion[LegNumber][i] = 0;
                FeetVelocity2BodyMotion[LegNumber][i] = 0;
            }

            for(i = 0; i < 3; i++)
            {
                SensorPosition[i] = KinematicParams[LegNumber][i];
            }
            
            Joint2HipFoot(Message,LegNumber);

            if(FootIsOnGround[LegNumber])
            {
                if(JointsXYZEnable){
                    ObservationCorrect_Position();

                    for(i = 0; i < 3; i++)
                    {
                        StateSpaceModel->Double_Par[0 + LegNumber * 3 + i] = Observation[3 * i]; 
                        FootBodyPos_WF[LegNumber][i] = Observation[3 * i];
                    }
                }

                if(JointsXYZVelocityEnable){
                    ObservationCorrect_Velocity();

                    for(i = 0; i < 3; i++)
                    {
                        FootBodyVel_WF[LegNumber][i] = Observation[3 * i + 1];
                    }
                }

                FeetEffort2Body(LegNumber);
                FeetVelocity2Body(Message, LegNumber);
            }

            for(i = 0; i < 6; i++)
            {
                StateSpaceModel->Double_Par[12+LegNumber*6+i] = FeetEffort2BodyMotion[LegNumber][i]; 
                StateSpaceModel->Double_Par[36+LegNumber*6+i] = FeetVelocity2BodyMotion[LegNumber][i]; 
            }
        }
        
        if(FootIsOnGround[0]||FootIsOnGround[1]||FootIsOnGround[2]||FootIsOnGround[3])
        {
            for(i = 0; i < 9; i++)
            {
                StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;
            }

            FootFallPositionRecord(Message);

            if(JointsXYZEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;
                }
            }
            if(JointsXYZVelocityEnable){
                for(i = 0; i < 3; i++)
                {
                    StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
                }
            }
            StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);

            // 前脚中心 与 后脚中心
            const double fx = 0.25 * (FootfallPositionRecord[0][0] + FootfallPositionRecord[1][0]);
            const double fy = 0.25 * (FootfallPositionRecord[0][1] + FootfallPositionRecord[1][1]);
            const double rx = 0.25 * (FootfallPositionRecord[2][0] + FootfallPositionRecord[3][0]);
            const double ry = 0.25 * (FootfallPositionRecord[2][1] + FootfallPositionRecord[3][1]);
            
            double x_mean = fx + rx, y_mean = fy + ry;

            double yaw_ff = std::atan2(fy - ry, fx - rx);

            angle_unwrap(yaw_ff);

            FootfallPar[0] = x_mean;
            FootfallPar[1] = y_mean;
            FootfallPar[2] = yaw_ff; 
        }
    }

    void SensorLegsPos::Joint2HipFoot(double *Message, int LegNumber)
    {
        double s1, s2, s3, c1, c2, c3, c23, s23, dq1, dq2, dq3;
        int SideSign, i;

        if (LegNumber == 0 || LegNumber == 2)
            SideSign = 1;
        else
            SideSign = -1;

        s1 = sin(Message[LegNumber*4+0]);
        s2 = sin(Message[LegNumber*4+1]);
        s3 = sin(Message[LegNumber*4+2]);
        c1 = cos(Message[LegNumber*4+0]);
        c2 = cos(Message[LegNumber*4+1]);
        c3 = cos(Message[LegNumber*4+2]);
        dq1 = Message[16 + LegNumber*4+0];
        dq2 = Message[16 + LegNumber*4+1];
        dq3 = Message[16 + LegNumber*4+2];

        c23 = c2 * c3 - s2 * s3;
        s23 = s2 * c3 + c2 * s3;

        Observation[0] = Par_CalfLength  * s23 + Par_ThighLength * s2;
        Observation[3] = Par_HipLength * SideSign * c1 + Par_CalfLength * (s1 * c23) + Par_ThighLength * c2 * s1;
        Observation[6] = Par_HipLength * SideSign * s1 - Par_CalfLength * (c1 * c23) - Par_ThighLength * c1 * c2 + Par_FootLength;

        Observation[1] = (Par_CalfLength *c23 + Par_ThighLength * c2)*dq2 + (Par_CalfLength *c23)*dq3;
        Observation[4] = (Par_CalfLength *c1*c23 + Par_ThighLength * c1*c2 - Par_HipLength*SideSign*s1)*dq1\
        + (-Par_CalfLength  * s1*s23 - Par_ThighLength * s1*s2)*dq2\
        + (-Par_CalfLength  * s1*s23)*dq3;
        Observation[7] = (Par_CalfLength *s1*c23 + Par_ThighLength * c2*s1 + Par_HipLength*SideSign*c1)*dq1\
        + (Par_CalfLength *c1*s23 + Par_ThighLength * c1*s2)*dq2\
        + (Par_CalfLength *c1*s23)*dq3;

        Observation[0] = -Observation[0];
        Observation[1] = -Observation[1];

        FootBodyPos_BF[LegNumber][0] = Observation[0] + SensorPosition[0];
        FootBodyPos_BF[LegNumber][1] = Observation[3] + SensorPosition[1];
        FootBodyPos_BF[LegNumber][2] = Observation[6] + SensorPosition[2];


        double tau_hip   = Message[32 + LegNumber * 4 + 0];
        double tau_thigh = Message[32 + LegNumber * 4 + 1];
        double tau_knee  = Message[32 + LegNumber * 4 + 2];

        double J[3][3];

        double Jx1_raw = 0.0;
        double Jx2_raw = Par_CalfLength * c23 + Par_ThighLength * c2;
        double Jx3_raw = Par_CalfLength * c23;

        double Jy1 = Par_CalfLength * c1 * c23 + Par_ThighLength * c1 * c2 - Par_HipLength * SideSign * s1;
        double Jy2 = -Par_CalfLength * s1 * s23 - Par_ThighLength * s1 * s2;
        double Jy3 = -Par_CalfLength * s1 * s23;

        double Jz1 = Par_CalfLength * s1 * c23 + Par_ThighLength * c2 * s1 + Par_HipLength * SideSign * c1;
        double Jz2 = Par_CalfLength * c1 * s23 + Par_ThighLength * c1 * s2;
        double Jz3 = Par_CalfLength * c1 * s23;

        J[0][0] = -Jx1_raw;
        J[0][1] = -Jx2_raw;
        J[0][2] = -Jx3_raw;

        J[1][0] =  Jy1;  J[1][1] =  Jy2;  J[1][2] =  Jy3;
        J[2][0] =  Jz1;  J[2][1] =  Jz2;  J[2][2] =  Jz3;


        const double tau[3] = { tau_hip, tau_thigh, tau_knee };
        // w = J * tau
        double w[3];
        mat3_mul_vec(J, tau, w);
        // M = J * J^T
        double M[3][3];
        mat3_mul_mat3T(J, M);
        // f = inv(M) * w
        double Minv[3][3];
        double f[3] = {0,0,0};
        if (mat3_inv(M, Minv))
            mat3_mul_vec(Minv, w, LatestFootEffort[LegNumber]);

        if(LatestFootEffort[LegNumber][2] <= FootEffortThreshold)
        {
            FootIsOnGround[LegNumber] = true;
        }
        else
        {
            FootIsOnGround[LegNumber] = false;
        }

        if(FootIsOnGround[LegNumber] && !FootWasOnGround[LegNumber])
        {
            // std::cout << "[Check] L" << LegNumber
            //     << " t=" << ObservationTime
            //     << " LatestFootEffort[LegNumber][2]=" << LatestFootEffort[LegNumber][2]
            //     << " FootEffortThreshold=" << FootEffortThreshold
            //     << " LastStatus=" << FootWasOnGround[LegNumber]
            //     << " LastMotion=" << FootLastMotion[LegNumber]
            //     << std::endl;
            FootLanding[LegNumber] = true;
            FootLastMotion[LegNumber] = true;
        }
        else
            FootLanding[LegNumber] = false;
        
        if(!FootIsOnGround[LegNumber] && FootWasOnGround[LegNumber])
        {
            FootLastMotion[LegNumber] = false;
        }

        if(LegNumber==3&&!FootIsOnGround[0]&&!FootIsOnGround[1]&&!FootIsOnGround[2]&&Observation[6]>-0.1)
            FootIsOnGround[LegNumber] = true;

        FootWasOnGround[LegNumber] = FootIsOnGround[LegNumber];
    }

    void SensorLegsPos::FootFallPositionRecord(double *Message){

        double p_sum[3] = {0}, v_sum[3] = {0};
        int    leg_cnt = 0;
        static double ShankPitchPrev[4] = {0,0,0,0};
        double body_roll=0.0, body_pitch=0.0, body_yaw=0.0;
        quat_to_eulerZYX(Est_Quaternion, body_roll, body_pitch, body_yaw);

        for (int LegNumber = 0; LegNumber < 4; ++LegNumber)
        {
            if (!FootIsOnGround[LegNumber])
                continue;
            if(!FootfallPositionRecordIsInitiated[LegNumber])
            {
                FootfallPositionRecordIsInitiated[LegNumber] = true;
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = 0;
                WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];
                ShankPitchPrev[LegNumber] = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];

            }
            else if(FootLanding[LegNumber])
            {
                FootLanding[LegNumber]= false;
                FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + FootBodyPos_WF[LegNumber][0];
                FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + FootBodyPos_WF[LegNumber][1];
                FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + FootBodyPos_WF[LegNumber][2];
                WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];
                ShankPitchPrev[LegNumber] = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];

                static double MapHeightStore[3][1000] = {0};
                static int MapHeightStoreMax = 0;
                int i = 0;
                double Zdifference = 99;
                
                // std::cout << "[LAND] L" << LegNumber
                //     << " t=" << ObservationTime
                //     << " z_in=" << (StateSpaceModel->EstimatedState[6] + Observation[6])
                //     << " scope=" << Environement_Height_Scope
                //     << " fade=" << Data_Fading_Time
                //     << " max=" << MapHeightStoreMax
                //     << std::endl;
            
                for(i = 0; i < (MapHeightStoreMax+1); i++)
                {
                    if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > Data_Fading_Time)
                    {
                        MapHeightStore[0][i] = 0;
                        MapHeightStore[1][i] = 0;
                        MapHeightStore[2][i] = 0;
                        // std::cout <<"One old step cleared" << std::endl;
                }
                }

                for(i = 0; i < (MapHeightStoreMax+1); i++){

                    if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope)
                    {
                        // std::cout << "[HIT] i=" << i
                        //         << " rec=" << MapHeightStore[0][i]
                        //         << " in="  << FootfallPositionRecord[LegNumber][2]
                        //         << " dz="  << std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2])
                        //         << " conf->" << MapHeightStore[1][i]
                        //         << std::endl;

                        MapHeightStore[1][i] *= exp(- (ObservationTime - MapHeightStore[2][i]) / (10 * Data_Fading_Time));
                        MapHeightStore[1][i] += 1;
                        MapHeightStore[2][i] = ObservationTime;
                        if(std::abs(MapHeightStore[0][i] - FootfallPositionRecord[LegNumber][2]) <= Environement_Height_Scope/10)
                            Zdifference = 0;
                        else
                            Zdifference = FootfallPositionRecord[LegNumber][2] - MapHeightStore[0][i];

                        // std::cout << "[CORR] z_diff=" << Zdifference
                        //     << " z_out=" << (FootfallPositionRecord[LegNumber][2] - Zdifference)
                        //     << std::endl;
                        break;
                    }
                }
                if(Zdifference == 99){
                    Zdifference = 0;
                    for(i = 0; i < (MapHeightStoreMax+1); i++)
                    {
                        if(MapHeightStore[2][i] == 0)
                        {
                            MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                            MapHeightStore[1][i] = 1;
                            MapHeightStore[2][i] = ObservationTime;
                            // std::cout << "[NEW] i=" << i
                            //     << " h=" << MapHeightStore[0][i]
                            //     << " t=" << MapHeightStore[2][i]
                            //     << " max=" << MapHeightStoreMax
                            //     << std::endl;
                            break;
                        }
                    }
                    if(i >= 999)
                    {
                        // std::cout << "[FULL] store full-ish, overwrite slot0"
                        //     << " t=" << ObservationTime
                        //     << " max=" << MapHeightStoreMax
                        //     << std::endl;

                        for(i = 0; i < (MapHeightStoreMax+1); i++)
                        {
                            if(MapHeightStore[2][i] != 0 && std::abs(ObservationTime-MapHeightStore[2][i]) > 60)
                            {
                                MapHeightStore[0][i] = 0;
                                MapHeightStore[1][i] = 0;
                                MapHeightStore[2][i] = 0;
                            }
                        }
                        i = 0;
                        MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                        MapHeightStore[1][i] = 1;
                        MapHeightStore[2][i] = ObservationTime;
                    }
                    if(i == MapHeightStoreMax + 1)
                    {
                        MapHeightStoreMax = i;
                        MapHeightStore[0][i] = FootfallPositionRecord[LegNumber][2];
                        MapHeightStore[1][i] = 1;
                        MapHeightStore[2][i] = ObservationTime;
                    }
                    
                } 
                FootfallPositionRecord[LegNumber][2] = FootfallPositionRecord[LegNumber][2] - Zdifference;
            }

            // 轮子转动角度
            double WheelRotationAngle = Message[LegNumber*4 + 3] - WheelAnglePrev[LegNumber];
            if (WheelRotationAngle >  2.0*M_PI) WheelRotationAngle -= 4.0*M_PI;
            if (WheelRotationAngle < -2.0*M_PI) WheelRotationAngle += 4.0*M_PI;
            WheelAnglePrev[LegNumber] = Message[LegNumber*4 + 3];

            // 小腿摆动角
            double ShankPitch = body_pitch + Message[LegNumber*4 + 1] + Message[LegNumber*4 + 2];
            double ShankRotationAngle = ShankPitch - ShankPitchPrev[LegNumber];
            if (ShankRotationAngle >  M_PI) ShankRotationAngle -= 2.0*M_PI;
            if (ShankRotationAngle < -M_PI) ShankRotationAngle += 2.0*M_PI;
            ShankPitchPrev[LegNumber] = ShankPitch;

            // 轮子有效转动角
            const double WheelRotationAngleEff = WheelRotationAngle - ShankRotationAngle;

            double hx = 1.0 - 2.0*(Est_Quaternion[2]*Est_Quaternion[2] + Est_Quaternion[3]*Est_Quaternion[3]);
            double hy = 2.0*(Est_Quaternion[1]*Est_Quaternion[2] + Est_Quaternion[0]*Est_Quaternion[3]);

            const double hn = std::sqrt(hx*hx + hy*hy);
            if (hn > 1e-9) {
                FootfallPositionRecord[LegNumber][0] += Par_FootLength * WheelRotationAngleEff * hx / hn;
                FootfallPositionRecord[LegNumber][1] += Par_FootLength * WheelRotationAngleEff * hy / hn;
            }

            // 轮对地有效角速度
            const double WheelRotationVelocityEff = Message[16 + LegNumber*4 + 3] - (Message[16 + LegNumber*4 + 1] + Message[16 + LegNumber*4 + 2]); 

            p_sum[0] += FootfallPositionRecord[LegNumber][0] - FootBodyPos_WF[LegNumber][0];
            p_sum[1] += FootfallPositionRecord[LegNumber][1] - FootBodyPos_WF[LegNumber][1];
            p_sum[2] += FootfallPositionRecord[LegNumber][2] - FootBodyPos_WF[LegNumber][2];

            v_sum[0] += FootBodyVel_WF[LegNumber][0] + Par_FootLength * WheelRotationVelocityEff * hx / hn;
            v_sum[1] += FootBodyVel_WF[LegNumber][1] + Par_FootLength * WheelRotationVelocityEff * hy / hn;
            v_sum[2] += FootBodyVel_WF[LegNumber][2];
            
            leg_cnt++;
        }

        Observation[0] = p_sum[0] / (double)leg_cnt;
        Observation[3] = p_sum[1] / (double)leg_cnt;
        Observation[6] = p_sum[2] / (double)leg_cnt;
        Observation[1] = v_sum[0] / (double)leg_cnt;
        Observation[4] = v_sum[1] / (double)leg_cnt;
        Observation[7] = v_sum[2] / (double)leg_cnt;
    }
    
    void SensorLegsPos::FeetEffort2Body(int LegNumber)
    {
        // -------- body frame: torque = r x f --------
        const double tau_b[3] = {
            FootBodyPos_BF[LegNumber][1]*LatestFootEffort[LegNumber][2] - FootBodyPos_BF[LegNumber][2]*LatestFootEffort[LegNumber][1],  // Mx  (roll moment about x)
            FootBodyPos_BF[LegNumber][2]*LatestFootEffort[LegNumber][0] - FootBodyPos_BF[LegNumber][0]*LatestFootEffort[LegNumber][2],  // My  (pitch moment about y)
            FootBodyPos_BF[LegNumber][0]*LatestFootEffort[LegNumber][1] - FootBodyPos_BF[LegNumber][1]*LatestFootEffort[LegNumber][0]   // Mz  (yaw moment about z)
        };

        // -------- world frame: rotate both --------
        double f_w[3], tau_w[3];
        quat_rot_vec3(Est_Quaternion, LatestFootEffort[LegNumber],   f_w);    // body -> world
        quat_rot_vec3(Est_Quaternion, tau_b, tau_w);  // body -> world

        // 3) 写入：XYZ 力 + XYZ 力矩（把它当作 roll/pitch/yaw 的力矩分量）
        FeetEffort2BodyMotion[LegNumber][0] = f_w[0];
        FeetEffort2BodyMotion[LegNumber][1] = f_w[1];
        FeetEffort2BodyMotion[LegNumber][2] = f_w[2];
        FeetEffort2BodyMotion[LegNumber][3] = tau_w[0];
        FeetEffort2BodyMotion[LegNumber][4] = tau_w[1];
        FeetEffort2BodyMotion[LegNumber][5] = tau_w[2];
    }
    
    void SensorLegsPos::FeetVelocity2Body(double *Message, int LegNumber)
    {
        // 轮缘线速度标量
        const double vmag = Par_FootLength * Message[16 + LegNumber * 4 + 3]; // m/s

        // ===== 取机身朝向方向：body x-axis -> world，投影到地面 =====
        double ex_b[3] = {1.0, 0.0, 0.0};
        double ex_w[3];
        quat_rot_vec3(Est_Quaternion, ex_b, ex_w);

        // 投影到XY平面并归一化
        double hx = ex_w[0];
        double hy = ex_w[1];
        const double hn = std::sqrt(hx*hx + hy*hy);
        if (hn < 1e-9) return;
        hx /= hn; hy /= hn;

        // 轮子在世界系的“目标速度向量”（平行地面）
        const double vix = vmag * hx;
        const double viy = vmag * hy;

        // ===== 用足点世界坐标：FootfallPositionRecord 是世界系接触点 =====
        const double xi = FootfallPositionRecord[LegNumber][0];
        const double yi = FootfallPositionRecord[LegNumber][1];

        // 机身参考点世界坐标（用你状态里的 x,y）
        const double x0 = StateSpaceModel->EstimatedState[0];
        const double y0 = StateSpaceModel->EstimatedState[3];

        const double dx = xi - x0;
        const double dy = yi - y0;

        // ===== 最小范数伪解：u=[Vx,Vy,wz]，A u = b
        // A = [ 1 0 -dy
        //       0 1  dx ]
        // u = A^T (A A^T)^-1 b
        //
        // AAT = [ 1+dy^2   -dy*dx
        //        -dy*dx     1+dx^2 ]
        const double a = 1.0 + dy*dy;
        const double b12 = -dy*dx;
        const double d = 1.0 + dx*dx;
        const double det = a*d - b12*b12;
        if (std::fabs(det) < 1e-12) return;

        // inv(AAT) * [vix; viy]
        const double y1 = ( d * vix - b12 * viy) / det;
        const double y2 = (-b12 * vix + a   * viy) / det;

        // u = A^T * [y1;y2]
        const double Vx = y1;
        const double Vy = y2;
        const double wz = (-dy) * y1 + (dx) * y2;

        // ===== 写入世界系：XYZ 速度 + XYZ 角速度（只给 yaw） =====
        FeetVelocity2BodyMotion[LegNumber][0] = Vx;
        FeetVelocity2BodyMotion[LegNumber][1] = Vy;
        FeetVelocity2BodyMotion[LegNumber][2] = 0.0;

        FeetVelocity2BodyMotion[LegNumber][3] = 0.0;
        FeetVelocity2BodyMotion[LegNumber][4] = 0.0;
        FeetVelocity2BodyMotion[LegNumber][5] = wz;
    }

    void SensorLegsPos::LoadedWeightCheck(double* Message, double Time) 
    {
        static int    w_state  = 0;
        static double z0_store = 0.0;
        static double wait3seconds  = 0;
        static double buf300[300] = {0.0};
        static int    buf300_i = 0;
        static int    buf300_n = 0;
        
        if(!CalculateWeightEnable){
            int k = 0;
            for(k = 32; k < 48; k++) 
                if (Message[k] != 0) 
                    break;
            if(k==48)
                CalculateWeightEnable = true;
        }

        if (CalculateWeightEnable) {
            if (w_state == 0) {
                z0_store = StateSpaceModel->EstimatedState[6];
                w_state = 1;
            }
            else if (w_state == 1) {
                if (StateSpaceModel->EstimatedState[6] - z0_store > 0.3) {
                    wait3seconds = Time;
                    w_state = 2;
                }
            }
            else if (w_state == 2) {
                if (Time - wait3seconds >= 3) {
                    buf300_i = 0;
                    buf300_n = 0;
                    w_state = 3;
                }
            }
            else if (w_state == 3) {
                buf300[buf300_i] = LatestFootEffort[0][2] + LatestFootEffort[1][2] + LatestFootEffort[2][2] + LatestFootEffort[3][2];
                buf300_i = (buf300_i + 1) % 300;
                buf300_n++;

                if (buf300_n >= 300) {
                    double s300 = 0.0;
                    for (int i = 0; i < 300; ++i) s300 += buf300[i];
                    double mean300 = s300 / 300.0;

                    LoadedWeight = - mean300 * 0.1;

                    if (LoadedWeight < 20.0) {
                        LoadedWeight = - DogWeight * 0.1;
                        mean300 = DogWeight;
                    }

                    FootEffortThreshold = mean300 * 0.2;

                    w_state  = 0;
                    
                    CalculateWeightEnable = false;
                }
            }
        }
    }

    void SensorLegsOri::SensorDataHandle(double* Message, double Time) 
    {
        if(!JointsRPYEnable)
            return;
        
        double P_body[4][3];
        double P_world[4][3];
        static double TimeRecord = Time;
        int LegNumber, i;

        for (LegNumber = 0; LegNumber < 4; ++LegNumber) {
            for (i = 0; i < 3; i++){
                P_body[LegNumber][i] = legs_pos_ref_->FootBodyPos_BF[LegNumber][i];
                P_world[LegNumber][i] = legs_pos_ref_->FootfallPositionRecord[LegNumber][i];
            }
        }

        int n_ground = 0;
        for (LegNumber = 0; LegNumber < 4; LegNumber++) {
            if (legs_pos_ref_->FootIsOnGround[LegNumber])
                n_ground++;
        }

        if (n_ground < 2) {
            return;
        }
        if (n_ground < 4){
            TimeRecord = Time;
            legori_current_weight = legori_init_weight;
        }
        else{
            legori_current_weight = (Time-TimeRecord) * (1.0 - legori_init_weight) /legori_time_weight + legori_init_weight;
            if(legori_current_weight>1.0)
                legori_current_weight = 1.0;
        }

        const double roll  = StateSpaceModel->EstimatedState[0];
        const double pitch = StateSpaceModel->EstimatedState[3];

        double q_rp[4];
        eulerZYX_to_quat(roll, pitch, 0.0, q_rp);

        double sx = 0.0, sy = 0.0;
        for (i = 0; i < 4; ++i) {
            if(!legs_pos_ref_->FootIsOnGround[i])
                continue;

            for (int j = i + 1; j < 4; ++j) {
                if(!legs_pos_ref_->FootIsOnGround[j])
                    continue;

                double vb_x = P_body[j][0] - P_body[i][0];
                double vb_y = P_body[j][1] - P_body[i][1];
                double vb_z = P_body[j][2] - P_body[i][2];

                double v_body[3] = { vb_x, vb_y, vb_z };
                double v_rp[3];
                quat_rot_vec3(q_rp, v_body, v_rp);


                double vw_x = P_world[j][0] - P_world[i][0];
                double vw_y = P_world[j][1] - P_world[i][1];
                double vw_z = P_world[j][2] - P_world[i][2];

                const double ang_rp = std::atan2(v_rp[1], v_rp[0]);
                const double ang_w  = std::atan2(vw_y,     vw_x);

                const double yaw_ij = angle_wrap(ang_w - ang_rp);
                sx += std::cos(yaw_ij);
                sy += std::sin(yaw_ij);

                int k = i+j;
                if(i==0)
                    k--;

                // printf("%d: ang_rp-%lf; ang_w-%lf; yaw_ij-%lf \n",k,ang_rp,ang_w,yaw_ij);
            }
        }

        if (sx != 0.0 || sy != 0.0) {
            const double yaw_est = std::atan2(sy, sx);
            const double yaw_now = StateSpaceModel->EstimatedState[6];
            const double err     = angle_wrap(yaw_est - yaw_now);
            legori_correct = angle_wrap(yaw_now + legori_current_weight * err);
            UpdateEst_Quaternion();
        }
    }
}