#pragma once

#include "SensorBase.h"

namespace DataFusion
{
  class SensorLegsPos : public Sensors
  {
    public:

    SensorLegsPos(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_)
    {
      for(int i=0;i<4;i++)
      {
        FootIsOnGround[i] = 1;
        FootWasOnGround[i] = 1;
        FootLanding[i] = 0;
        FootLastMotion[i] = 1;
      }

      UseGo2P();
    }

    double KinematicParams[4][13];
    double Par_HipLength = 0, Par_ThighLength = 0, Par_CalfLength = 0, Par_FootLength = 0;

    // ========= 预设：Go2点足 =========
    static constexpr double Go2P_PARAM[4][13] = {
      {  0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      {  0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      { -0.1934, -0.0465,  0.000,   0.0, -0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 },
      { -0.1934,  0.0465,  0.000,   0.0,  0.0955,  0.0,   0.0,  0.0, -0.213,   0.0,  0.0, -0.213,  0.022 }
    };
    static constexpr double Go2P_HIP   = 0.0955;
    static constexpr double Go2P_THIGH = 0.213;
    static constexpr double Go2P_CALF  = 0.213 + 0.022;
    static constexpr double Go2P_FOOT  = 0.00;

    void UseGo2P()
    {
      std::memcpy(KinematicParams, Go2P_PARAM, sizeof(KinematicParams));
      Par_HipLength = Go2P_HIP; Par_ThighLength = Go2P_THIGH; Par_CalfLength = Go2P_CALF; Par_FootLength = Go2P_FOOT;
    }

    bool JointsXYZEnable = true;
    bool JointsXYZVelocityEnable = false;

    void SensorDataHandle(double* Message, double Time)  override;

    double FootEffortThreshold = -1.0, Environement_Height_Scope = 0.08, Data_Fading_Time = 600.0;
    bool FootfallPositionRecordIsInitiated[4] = {0}, FootIsOnGround[4] = {0}, FootWasOnGround[4] = {0}, FootLastMotion[4] = {0}, FootLanding[4] = {0}, CalculateWeightEnable = false;
    double LatestFeetEffort[4]={0};
    double FootBodyPosition[4][3]={0}, FootfallPositionRecord[4][3]={0};

    protected:

    void Joint2HipFoot(double *Message, int LegNumber);
    void PositionCorrect(int LegnNmber);

  };

  class SensorLegsOri : public Sensors
  {
    public:

    SensorLegsOri(EstimatorPortN* StateSpaceModel_):Sensors(StateSpaceModel_){}
    void SensorDataHandle(double* Message, double Time) override;

    inline void SetLegsPosRef(class SensorLegsPos* ref){ legs_pos_ref_ = ref; }
    double legori_init_weight = 0.001, legori_time_weight = 1000.0, legori_current_weight = 0.001, legori_correct = 0;

    bool JointsRPYEnable = false;

    protected:

    class SensorLegsPos* legs_pos_ref_ = nullptr;
    
  };

}