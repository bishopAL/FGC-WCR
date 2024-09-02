#pragma once
#include "leg.h"
using namespace Eigen;
using namespace std;
extern bool swingtimeFlag;
class CGebot{

public:
    float m_fLength,m_fWidth,m_fHeight,m_fMass,m_fIxx,m_fIyy,m_fIzz; //2*(distance from com to shoulder);
    float m_threhold[4];
    CLeg* m_glLeg[4];


public:
    float fTimeForSwing[4];
    bool bInitFlag;
    float fTimeForGaitPeriod;  // The time of the whole period
    float fTimePeriod;  // The time of one period
    float fTimePresent;
    Matrix<float, 4, 2> mfTimeForSwingPhase;  // startTime, endTime: LF, RF, LH, RH
    Matrix3f Ic[8]; // interia of Leg
    Matrix3f Ic_body;//interia of body
    Vector3f Pc[8]; //com of each link ;relate to pre aixs;
    Vector3f Pc_body;//com of body ,relate to base aixs;
    float fSwingStatus[4][3];  //swing status,dettach - swing -attach , sum = 1;
    float fSwingPhaseStatusPart[4];   //swing phase status, dettach - swingUp - swingDown - attach , sum = 1;
    float fStancePhaseStatusPart[2];  // //stance phase status, recover - stance , sum = 1;
    int iStatusCounter[4], iStatusCounterBuffer[4][6];
    float Tsu,Tst; //time of swing up and stance,its fixed
    float v_swingDown,v_rec ;
    float fStepHeight;
    float attchDis[4];
    int attchTimes[4];
    bool touchTrigger[4],probeTrigger[4],autoControlFlag;
    
    Matrix<float, 6,1> vfTargetCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 6,1> vfPresentCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 6,1> vfLastCoMVelocity;  // X, Y , Z ,yaw in world cordinate
    Matrix<float, 3, 1> vfTargetCoMPosition;  // com X, Y , Z in world cordinate
    Matrix<float, 3, 1> vfTargetCoMPosture; //roll , pitch , yaw ,in world cordinate
    //raw code :
    Matrix<float, 4, 2> mfTimeForStancePhase;
    Matrix<float, 4, 3> targetCoMPosition;
    Matrix<float, 4, 3> targetCoMPosture;
    //end 
    Matrix<float, 4, 1> mfTimePresentForSwing;
    Matrix<float, 4, 4> mfShoulderPos;  // X-Y: LF, RF, LH, RH
    Matrix<float, 4, 3> mfStancePhaseStartPos;
    Matrix<float, 4, 3> mfStancePhaseEndPos;
    Matrix<float, 4, 3> mfInitFootPos;    // initial position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegCmdPos;  // command position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegPresPos;  // present position of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegLastPos;
    Matrix<float, 4, 3> mfLegPresVel;  // present velocity of foot  in shoulder coordinate, in order LF, RF, LH, RH; X-Y-Z
    Matrix<float, 4, 3> mfLegLastVel;   //last velocity of foot to shoulder,for filter
    Matrix<float, 4, 3> mfJointCmdPos;  // command joint angle 0-11
    Matrix<float, 4, 4> mfJointPresPos;  // present motor 0-16
    Matrix<float, 4, 4> mfJointLastPos;  // present motor 0-16
    Matrix<float, 4, 4> mfJointPresVel;  // present motor 0-16
    Matrix<float, 4, 4> mfJointCmdVel; 
    Matrix<float, 1, 3> mfSwingVelocity;
    Matrix<float, 4, 3> mfJointCompDis;  // compensation joint angle
    Matrix<float, 4, 3> mfCompensation;
    Matrix<float, 4, 3> mfLegCmdCompPos;
    Matrix<float, 3, 1> mfCompensationZ;
    bool BSwingPhaseStartFlag, BSwingPhaseEndFlag;
    vector<float> vLastSetPos;
    CGebot(float length,float width,float height,float mass);
    void SetInitPos(Matrix<float, 4, 3> initPosition);
    void InertiaInit();
    void SetCoMVel(Matrix<float, 6,1> tCV);   
    void SetPhase(float tP, float tFGP, Matrix<float, 4, 2> tFSP);
    void NextStep();
    void UpdatejointPresPosAndVel(vector<float> present_position);
    void UpdatejointPresVel();
    void UpdateJacobians();
    void ForwardKinematics(int mode);
    void InverseKinematics(Matrix<float, 4, 3> cmdpos);   // standing state
    void UpdateFtsPresVel(); 
    void UpdateTouchStatus(vector<int> values,vector<int> prevalues,vector<int> preprevalues);
    //robot control
    //pump control
     uint8_t svStatus=0b00000000;
     API api;
    void AirControl();
    void PumpNegtive(int legNum);
    void PumpPositive(int legNum);
    void PumpAllNegtive();
    void PumpAllPositve();
    void PumpAllClose();
    Matrix<float, 3, 1> fnn_outputcpt(VectorXd vec);
    void fnn_stepmodify();
    //motor control
     
    //DxlAPI dxlMotors;  //3000000  cannot hold 6 legs ttyUSB0 ttyAMA0
    
   
    //void SetTor(vector<float> setTor);
    
    void UpdateLegStatus(int legNum);
    void AttitudeCorrection180();
    void AttitudeCorrection90();
     
    

};
