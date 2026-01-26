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
        
        for(int i = 0; i < StateSpaceModel->Nz; i++)
            Observation[i] = 0;

        for(LegNumber = 0; LegNumber<4; LegNumber++)
        {
            LatestFeetEffort[LegNumber] = 0;

            for(i = 0; i < 3; i++)
            {
                SensorPosition[i] = KinematicParams[LegNumber][i];
            }
            
            Joint2HipFoot(Message,LegNumber);

            if(FootIsOnGround[LegNumber])
            {
                for(i = 0; i < 9; i++)
                {
                    StateSpaceModel->Matrix_H[i * StateSpaceModel->Nx + i] = 0;
                }
                if(JointsXYZEnable){
                    ObservationCorrect_Position();
                    
                    for(i = 0; i < 3; i++)
                        StateSpaceModel->Matrix_H[(3 * i + 0) * StateSpaceModel->Nx + (3 * i + 0)] = 1;

                    PositionCorrect(LegNumber);
                }

                if(JointsXYZVelocityEnable){
                    ObservationCorrect_Velocity();

                    for(i = 0; i < 3; i++)
                        StateSpaceModel->Matrix_H[(3 * i + 1) * StateSpaceModel->Nx + (3 * i + 1)] = 1;
                }
                
                StateSpaceModel_Go2_EstimatorPort(Observation, ObservationTime, StateSpaceModel);
            }
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

        s1 = sin(Message[LegNumber*3+0]);
        s2 = sin(Message[LegNumber*3+1]);
        s3 = sin(Message[LegNumber*3+2]);
        c1 = cos(Message[LegNumber*3+0]);
        c2 = cos(Message[LegNumber*3+1]);
        c3 = cos(Message[LegNumber*3+2]);
        dq1 = Message[12 + LegNumber*3+0];
        dq2 = Message[12 + LegNumber*3+1];
        dq3 = Message[12 + LegNumber*3+2];

        c23 = c2 * c3 - s2 * s3;
        s23 = s2 * c3 + c2 * s3;

        Observation[0] = Par_CalfLength  * s23 + Par_ThighLength * s2;
        Observation[3] = Par_HipLength * SideSign * c1 + Par_CalfLength * (s1 * c23) + Par_ThighLength * c2 * s1;
        Observation[6] = Par_HipLength * SideSign * s1 - Par_CalfLength * (c1 * c23) - Par_ThighLength * c1 * c2;

        Observation[1] = (Par_CalfLength *c23 + Par_ThighLength * c2)*dq2 + (Par_CalfLength *c23)*dq3;
        Observation[4] = (Par_CalfLength *c1*c23 + Par_ThighLength * c1*c2 - Par_HipLength*SideSign*s1)*dq1\
        + (-Par_CalfLength  * s1*s23 - Par_ThighLength * s1*s2)*dq2\
        + (-Par_CalfLength  * s1*s23)*dq3;
        Observation[7] = (Par_CalfLength *s1*c23 + Par_ThighLength * c2*s1 + Par_HipLength*SideSign*c1)*dq1\
        + (Par_CalfLength *c1*s23 + Par_ThighLength * c1*s2)*dq2\
        + (Par_CalfLength *c1*s23)*dq3;

        Observation[0] = -Observation[0];
        Observation[1] = -Observation[1];

        FootBodyPosition[LegNumber][0] = Observation[0] + SensorPosition[0];
        FootBodyPosition[LegNumber][1] = Observation[3] + SensorPosition[1];
        FootBodyPosition[LegNumber][2] = Observation[6] + SensorPosition[2];


        double tau_hip   = Message[24 + LegNumber * 3 + 0];
        double tau_thigh = Message[24 + LegNumber * 3 + 1];
        double tau_knee  = Message[24 + LegNumber * 3 + 2];

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
        if (mat3_inv(M, Minv)) {
            mat3_mul_vec(Minv, w, f);
            LatestFeetEffort[LegNumber] = f[2];
        }

        if(LatestFeetEffort[LegNumber] <= FootEffortThreshold)
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
            //     << " LatestFeetEffort[LegNumber]=" << LatestFeetEffort[LegNumber]
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

    void SensorLegsPos::PositionCorrect(int LegNumber){

        if(!FootfallPositionRecordIsInitiated[LegNumber])
        {
            FootfallPositionRecordIsInitiated[LegNumber] = true;
            FootLanding[LegNumber]= false;
            FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + Observation[0];
            FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + Observation[3];
            FootfallPositionRecord[LegNumber][2] = 0;
        }
        else if(FootLanding[LegNumber])
        {
            FootLanding[LegNumber]= false;
            FootfallPositionRecord[LegNumber][0] = StateSpaceModel->EstimatedState[0] + Observation[0];
            FootfallPositionRecord[LegNumber][1] = StateSpaceModel->EstimatedState[3] + Observation[3];
            FootfallPositionRecord[LegNumber][2] = StateSpaceModel->EstimatedState[6] + Observation[6];

            static double MapHeightStore[3][1000] = {0};
            static int MapHeightStoreMax = 0;
            int i = 0;
            double distance = 0, Zdifference = 99, Temp[4] = {0,0,0,0};
            double AngleA = atan(std::abs(Observation[3]) / std::abs(Observation[0]));

            
            // std::cout << "[LAND] L" << LegNumber
            //     << " t=" << ObservationTime
            //     << " z_in=" << (StateSpaceModel->EstimatedState[6] + Observation[6])
            //     << " scope=" << Environement_Height_Scope
            //     << " fade=" << Data_Fading_Time
            //     << " max=" << MapHeightStoreMax
            //     << std::endl;
        
            distance = std::sqrt(Observation[0]*Observation[0] + Observation[3]*Observation[3] + Observation[6]*Observation[6]);
        
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

        Observation[0] = FootfallPositionRecord[LegNumber][0] - Observation[0];
        Observation[3] = FootfallPositionRecord[LegNumber][1] - Observation[3];
        Observation[6] = FootfallPositionRecord[LegNumber][2] - Observation[6];
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
                P_body[LegNumber][i] = legs_pos_ref_->FootBodyPosition[LegNumber][i];
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