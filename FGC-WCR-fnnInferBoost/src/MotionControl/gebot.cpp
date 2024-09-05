#include "gebot.h"


bool swingtimeFlag = false;
CGebot::CGebot(float length,float width,float height,float mass)
{
    //dxlMotors.init("/dev/ttyAMA0", 1000000, ID, 2);  // CAN NOT 4M.   ttyUSB0 ttyAMA0      
    m_glLeg[0] = new CLeg(LF,60.0,60.0,30.0);  // mm
    m_glLeg[1] = new CLeg(RF,60.0,60.0,30.0);
    m_glLeg[2] = new CLeg(LH,60.0,60.0,30.0);
    m_glLeg[3] = new CLeg(RH,60.0,60.0,30.0); //24
    m_fLength=length/1000.0;
    m_fWidth=width/1000.0;
    m_fHeight=height/1000.0;
    m_fMass=mass/1000.0; //g->kg
    m_threhold[0]=THREHOLDLF;
    m_threhold[1]=THREHOLDRF;
    m_threhold[2]=THREHOLDLH;
    m_threhold[3]=THREHOLDRH;
    mfShoulderPos<<m_fWidth/2, m_fLength/2, 0,1,m_fWidth/2, -m_fLength/2,0,1, -m_fWidth/2, m_fLength/2,0,1,-m_fWidth/2, -m_fLength/2,0,1;  // X-Y: LF, RF, LH, RH
    mfShoulderPosCompensation<<m_fWidth/2, m_fLength/2, 0,m_fWidth/2, -m_fLength/2,0,-m_fWidth/2, m_fLength/2,0,-m_fWidth/2, -m_fLength/2,0;
    for (size_t i = 0; i < 4; i++)
    {
        touchTrigger[i]=false;
        probeTrigger[i]=false;
        attchTimes[i]=ATTACH_TIMES;
    }
    
    fSwingPhaseStatusPart[0]=0.4; //detach
    fSwingPhaseStatusPart[1]=0.3; //swingUp
    fSwingPhaseStatusPart[2]=0.3; //swingDown
    // fSwingPhaseStatusPart[3]=0.3; //attach
    fStancePhaseStatusPart[0]=0.1;//recover  
    fStancePhaseStatusPart[1]=0.9;//stance
    mfSwingVelocity.setZero();
    targetCoMPosture.setZero();
    BSwingPhaseStartFlag = true;
    autoControlFlag=true;
    BSwingPhaseEndFlag = 0;     //
    mfCompensation.setZero();
    mfCompensationZ.setZero();
    mfJointCompDis.setZero();
    mfLegCmdCompPos.setZero();

    for(int i=0;i<16;++i)
        vLastSetPos.push_back(0);
    fTimePresent=0.0;
    mfTimePresentForSwing.setZero();
    vfTargetCoMVelocity.setZero();
    //dxlMotors.setOperatingMode(3);  //3 position control; 0 current control
    usleep(500);
   // dxlMotors.torqueEnable();
    //dxlMotors.getPosition();
    // api.setPump(1, LOW);//LF
    // api.setPump(24, LOW);//RF
    // api.setPump(28, LOW);//LH
    // api.setPump(29, LOW);//RH
    // api.setPump(1, HIGH);//LF
    // api.setPump(24, HIGH);//RF
    // api.setPump(28, HIGH);//LH
    // api.setPump(29, HIGH);//RH
    usleep(1e6);
}


/**
 * @brief set phases for gait
 * 
 * @param tP The time of one period
 * @param tFGP The time of the whole period
 * @param tFSP The time of stance phase on start and end, in order LF, RF, LH, RH
 */
void CGebot::SetPhase(float tP, float tFGP, Matrix<float,4,2> tFSP)
{
    float fSwPSFactor[4], fStPSFactor[2];
    fTimePeriod = tP;
    fTimeForGaitPeriod = tFGP;
    mfTimeForSwingPhase = tFSP;
    fTimePresent = 0.0;
    // mfTimeForStancePhase<< TimeForGaitPeriod/4.0 *3,          TimeForGaitPeriod/4.0 *2,   // tripod
    //                     TimeForGaitPeriod/4.0,             TimeForGaitPeriod,
    //                     TimeForGaitPeriod - TimePeriod,    TimeForGaitPeriod/4.0 *3,
    //                     TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0;
    mfTimeForStancePhase<<mfTimeForSwingPhase(0,1), mfTimeForSwingPhase(0,0),
                        mfTimeForSwingPhase(1,1), mfTimeForSwingPhase(1,0),
                        mfTimeForSwingPhase(2,1), mfTimeForSwingPhase(2,0),
                        mfTimeForSwingPhase(3,1), mfTimeForSwingPhase(3,0);
    mfTimePresentForSwing.setZero();

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        fTimeForSwing[legNum] = mfTimeForSwingPhase(legNum,1)-mfTimeForSwingPhase(legNum,0);
        iStatusCounterBuffer[legNum][int(detach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(swingUp)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[1]);
        iStatusCounterBuffer[legNum][int(swingDown)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[2]);
        // iStatusCounterBuffer[legNum][int(attach)] = floor(fTimeForSwing[legNum] / fTimePeriod * fSwingPhaseStatusPart[3]);
        iStatusCounterBuffer[legNum][int(recover)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[0]);
        iStatusCounterBuffer[legNum][int(stance)] = floor( (fTimeForGaitPeriod - fTimeForSwing[legNum]) / fTimePeriod * fStancePhaseStatusPart[1]);
    }
    
    fSwPSFactor[0]=fSwingPhaseStatusPart[0];
    fSwPSFactor[1]=fSwPSFactor[0]+fSwingPhaseStatusPart[1];
    fSwPSFactor[2]=fSwPSFactor[1]+fSwingPhaseStatusPart[2];
    // fSwPSFactor[3]=fSwPSFactor[2]+fSwingPhaseStatusPart[3];
    fStPSFactor[0]=fStancePhaseStatusPart[0];
    fStPSFactor[1]=fStPSFactor[0]+fStancePhaseStatusPart[1];
    for(uint8_t legNum=0; legNum<4; legNum++) 
    {
        if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  0 &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[0] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[0] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(detach);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[0] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[1] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[1] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingUp);
        }
        else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[1] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[2] )
		{
            iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[2] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
            m_glLeg[legNum]->ChangeStatus(swingDown);
        }
        // else if(fTimePresent - mfTimeForSwingPhase(legNum,0) >=  fTimeForSwing[legNum] * fSwPSFactor[2] &&  fTimePresent - mfTimeForSwingPhase(legNum,0) <  fTimeForSwing[legNum] * fSwPSFactor[3] )
		// {
        //     iStatusCounter[legNum] = floor((fTimeForSwing[legNum] * fSwPSFactor[3] - (fTimePresent-mfTimeForSwingPhase(legNum,0)) ) / fTimePeriod);
        //     m_glLeg[legNum]->ChangeStatus(attach);
        // }
        else //stance phase
		{
            if( fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0) < (fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] )
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[0] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0)))/ fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(recover);
            }
            else 
            {
                iStatusCounter[legNum] = floor(((fTimeForGaitPeriod - fTimeForSwing[legNum]) * fStPSFactor[1] - (fTimePresent + fTimeForGaitPeriod - mfTimeForStancePhase(legNum,0))) / fTimePeriod);
                m_glLeg[legNum]->ChangeStatus(stance);
            }
        }
    }  

}

/**
 * @brief Update the status of leg, record the start Pos and the end Pos of stance phase.
 * 
 * @param legNum the leg number
 */
void CGebot::UpdateLegStatus(int legNum)
{
     if(m_glLeg[legNum]->GetLegStatus()==attach)
    {
        attchTimes[legNum]--;
        if(attchTimes[legNum]<0){
            autoControlFlag=0;
            return;
        }
    }
    else
     iStatusCounter[legNum]--;
   
    // BSwingPhaseStartFlag = 0;
    // BSwingPhaseEndFlag = 0;
    bool ts=m_glLeg[legNum]->getTouchStatus() ;
   
    if(probeTrigger[legNum]== true){
        if(ts == true){
              probeTrigger[legNum]=false;
              touchTrigger[legNum]=true;
        }
    }
    if(iStatusCounter[legNum] <= 0 || touchTrigger[legNum] == true )
        switch(m_glLeg[legNum]->GetLegStatus())
        {
        case detach:
            m_glLeg[legNum]->ChangeStatus(swingUp);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingUp)];
            break;
        case swingUp:
            m_glLeg[legNum]->ChangeStatus(swingDown);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(swingDown)];
            break;
        case swingDown:
            if(ts != true){
                m_glLeg[legNum]->ChangeStatus(attach);
                iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
                probeTrigger[legNum]=true;
            }
            else{
            swingtimeFlag=true;
            m_glLeg[legNum]->ChangeStatus(recover);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
            mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
            for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            BSwingPhaseEndFlag = true;
            }
            break;
        case attach:
            m_glLeg[legNum]->ChangeStatus(recover);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(recover)];
            mfStancePhaseStartPos(legNum) = mfLegCmdPos(legNum);
            for(int pos=0; pos<3; pos++)
                    targetCoMPosition(legNum, pos) = 0.0;
            BSwingPhaseEndFlag = true;
            touchTrigger[legNum]=false;
            attchTimes[legNum]=ATTACH_TIMES;
            break;
        case recover:
            m_glLeg[legNum]->ChangeStatus(stance);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(stance)]-attchDis[legNum];
            attchDis[legNum]=0;
            break;
        case stance:
            m_glLeg[legNum]->ChangeStatus(detach);
            iStatusCounter[legNum] = iStatusCounterBuffer[legNum][int(detach)];
            mfStancePhaseEndPos(legNum) = mfLegCmdPos(legNum);
            mfSwingVelocity = -(mfStancePhaseEndPos.row(legNum) - mfStancePhaseStartPos.row(legNum)) / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]) ;
            BSwingPhaseStartFlag = true;
            break;
        }
}

/**
 * @brief set initial position of feet in shoulder coordinate
 * 
 * @param initPosition foot position in shoulder coordinate.
 * @note The lenth of legs, whitch is L1, L2, L3 in constructor of MotionControl.
 */

void CGebot::SetInitPos(Matrix<float, 4, 3> initPosition)
{
    mfStancePhaseStartPos = initPosition;
    mfStancePhaseEndPos = initPosition;
    mfLegPresPos = initPosition;
    mfLegCmdPos = initPosition;
    mfInitFootPos = initPosition;
    targetCoMPosition.setZero();
}
/**
 * @brief init inertia of robot 
 * @note ,Ic 0,1 rep mass of lf 1 and 23,then rf 2 3,lh 4 5, rh 6 7.at last init body Ic;
 *         same to Pc;
 * 
 */
void CGebot::InertiaInit()
{
    float  I[256];
    string2float("../include/inertiaInit.csv", I);
    for(size_t i=0;i<96;i=i+12)
    {   
        Pc[i/12]<<I[i],I[i+1],I[i+2];
        Ic[i/12]<<I[i+3],I[i+4],I[i+5],
               I[i+6],I[i+7],I[i+8],
               I[i+9],I[i+10],I[i+11]; 
    }
    Pc_body<<-1.96,-0.22,10.32;
    Ic_body<<421534.86,-488.38,-36537.05,
             -488.38,1054973.20,-1750.03,
            -36537.05,-1750.03,1211574.07;
    //g*mm^2 2 kg*m^2;
    for (size_t i=0;i<8;i++)
    {
        Pc[i]=Pc[i]/1e3;
        //   cout<<"Pc:"<<endl;
        // cout<<Pc[i]<<endl;
    }
    for(size_t i=0;i<8;i++)
    {
        Ic[i]=Ic[i]/1e9;
        // cout<<"Ic:"<<endl;
        // cout<<Ic[i]<<endl;
    }
    Pc_body=Pc_body/1e3;
    Ic_body=Ic_body/1e9;
}

/**
 * @brief 
 * 
 * @param tCV 
 * set  Vel of X,Y,alpha in world cordinate
 */
void CGebot::SetCoMVel(Matrix<float, 6,1> tCV)
{   
    vfTargetCoMVelocity = tCV;
}


/**
 * @brief 
 * 
 * @param jointPos 
 * put (vector)jointPos[12] into (Matrix)jointPresPos(4,3)
 */
void CGebot::UpdatejointPresPosAndVel(vector<float> present_position)
{
     mfJointPresPos=inverseMotorMapping(present_position);

    // cout<<"mfJointPresPos: " <<mfJointPresPos<<endl;
    // cout<<"mfJointLastPos: " <<mfJointLastPos<<endl;
    for(int legNum=0;legNum<4;legNum++)
    {   
        Matrix<float,4,1> temp=mfJointPresPos.block(legNum,0,1,4).transpose();
        m_glLeg[legNum]->SetJointPos(temp);
    }

    mfJointPresVel=(mfJointPresPos-mfJointLastPos)*loopRateCommandUpdate;
    mfJointLastPos=mfJointPresPos;
  
}

/**
 * @brief 
 * 
 * @param jointVel 
 * put (vector)jointVel[12] into (Matrix)jointPresVel(4,3)
 */
void CGebot::UpdatejointPresVel()
{
    //mfJointPresVel=inverseMotorMapping(dxlMotors.present_velocity);    
    //cout<<"dxlMotors.present_velocity:  ";
    // for(size_t i=0;i<dxlMotors.present_velocity.size();i++)   
    // {
    //    // cout<<dxlMotors.present_velocity[i]<<" ";
    // }
   // cout<<endl;
    //cout<<"mfJointPresVel:\n"<<mfJointPresVel<<endl;
}

void CGebot::UpdateJacobians()
{
    for(int legNum=0;legNum<4;legNum++)
        m_glLeg[legNum]->UpdateJacobian();
}

/**
 * @brief 
 * update Vel of feet in shoulder coordinate
 */
void CGebot::UpdateFtsPresVel()
{
    // mfLegLastVel=mfLegPresVel;
    // Matrix <float, 3, 1> temp_vel;
    // for(int i=0; i<4; i++)
    // {
    //     temp_vel = m_glLeg[i]->GetJacobian() * mfJointPresVel.row(i).transpose();
    //     mfLegPresVel.row(i) = temp_vel.transpose();
    // }
    
}
void CGebot::NextStep()
{
    if (abs(fTimePresent - fTimeForGaitPeriod ) < 1e-4)  // check if present time has reach the gait period                                                               
    {                                                            // if so, set it to 0.0
        fTimePresent = 0.0;
        // legCmdPos = initFootPos;
    }

    for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
    {   
        UpdateLegStatus(legNum);
        enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus(); //get present status
        if(legNum<2)
            fStepHeight = StepHeight_F;
        else
            fStepHeight = StepHeight_H;
        
        // cout<<"leg_"<<(int)legNum<<"_status: "<<(int)ls<<endl;
        if( ls == stance || ls == recover) //stance phase
        {     
            for(int pos=0; pos<3; pos++)
            {
                targetCoMPosition(legNum, pos) += vfTargetCoMVelocity(pos,0)*fTimePeriod;
                targetCoMPosture(legNum,pos) +=vfTargetCoMVelocity(pos+3,0)*fTimePeriod;
            }
            Matrix<float, 4, 4> trans;
            trans<<cos(targetCoMPosture(legNum,2)), -sin(targetCoMPosture(legNum,2)),0, targetCoMPosition(legNum,0),
                sin(targetCoMPosture(legNum,2)), cos(targetCoMPosture(legNum,2)),0, targetCoMPosition(legNum,1),
                0,0,1,targetCoMPosition(legNum,2),
                0, 0, 0, 1;
            Matrix<float, 4, 1> oneShoulderPos_4x1;
            oneShoulderPos_4x1<<mfShoulderPos(legNum,0), mfShoulderPos(legNum,1),0,1;
            oneShoulderPos_4x1 = trans * oneShoulderPos_4x1;
            if(ls==stance)
             for (size_t i = 0; i < 3; i++)
                mfLegCmdPos(legNum, i) = mfStancePhaseStartPos(legNum, i) + (mfShoulderPos(legNum, i) - oneShoulderPos_4x1(i));
                //  cout<<"mfStancePhaseStartPos(legNum, i): \n"<<mfStancePhaseStartPos(legNum, i)<<endl;
                //  cout<<"oneShoulderPos_3x1("<<i<<")"<<oneShoulderPos_4x1(i)<<endl;
                //  cout<<"mfShoulderPos(legNum,"<<i<<")"<<mfShoulderPos(legNum, i)<<endl;
            else if(ls == recover){
                for (size_t i = 0; i < 3; i++)
                    mfLegCmdPos(legNum, i) = mfStancePhaseStartPos(legNum, i) + (mfShoulderPos(legNum, i) - oneShoulderPos_4x1(i));
                mfLegCmdPos(legNum, 2) +=  attchDis[legNum] / iStatusCounterBuffer[legNum][(int)ls];

            }
        }
        else if( ls == detach || ls == swingUp )   //swing phase 
        {
            // cout<<"swing-"<<(unsigned)legNum<<endl;
            float x, xh, m, n, k;
            if(mfSwingVelocity(0, 0) == 0)      //first step
            {
                mfLegCmdPos(legNum, 2) += fStepHeight / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp]);
                
            }
            else 
            {            
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos);     
                x = mfLegCmdPos(legNum, 0) - mfStancePhaseEndPos(legNum, 0);
                xh = -(mfStancePhaseEndPos(legNum, 0) - mfStancePhaseStartPos(legNum, 0)) * (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp])  / (iStatusCounterBuffer[legNum][(int)detach] + iStatusCounterBuffer[legNum][(int)swingUp] + iStatusCounterBuffer[legNum][(int)swingDown]);

                /* z = -( x - m )^2 + n + z0    */
                // m = (StepHeight + xh * xh) /2 /xh;
                // n = m * m;
                // legCmdPos(legNum, 2) = -(x - m) * (x - m) + n + stancePhaseEndPos(legNum, 2);

                /* z = -k * ( x - xh )^2 + H + z0 */
                k = fStepHeight / xh / xh;
                mfLegCmdPos(legNum, 2) = -k * (x - xh) * (x - xh) + fStepHeight + mfStancePhaseEndPos(legNum, 2);
               
            }
        }
        else if( ls == swingDown )    //swing phase
        {
            if(mfSwingVelocity( 0, 0) != 0)     
            {
                for(uint8_t pos=0; pos<2; pos++)
                    mfLegCmdPos(legNum, pos) += mfSwingVelocity(pos) ;
            }   
            
            mfLegCmdPos(legNum, 2) -= fStepHeight / iStatusCounterBuffer[legNum][(int)ls];
        }
        else if( ls == attach )    //swing phase
        {
            mfLegCmdPos(legNum, 0) -=vfTargetCoMVelocity(0,0)*fTimePeriod; // make x y stationary to ground;
            mfLegCmdPos(legNum, 1) -=vfTargetCoMVelocity(1,0)*fTimePeriod;
            mfLegCmdPos(legNum, 2) -=  ATTACHDIS_MAX/ATTACH_TIMES;
            attchDis[legNum]+=ATTACHDIS_MAX/ATTACH_TIMES;
        }
        // else if( ls == recover )   //stance phase
        // {   // mfLegCmdPos(legNum, 2)  is not related to mfStancePhaseEndPos(legNum, 2) and mfStartPhaseEndPos(legNum, 2)
        //     mfLegCmdPos(legNum, 2) +=  attchDis[legNum] / iStatusCounterBuffer[legNum][(int)ls];
        // }
        //cout<<"legNum_"<<(int)legNum<<":"<<stepFlag[legNum]<<"  ";
    }
   
    for(uint8_t legNum=0; legNum<4; legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()!= stance) mfTimePresentForSwing(legNum) += fTimePeriod;
        else mfTimePresentForSwing(legNum) = 0;   //stance phase
    }
    fTimePresent += fTimePeriod;
}

/**
 * @brief forwardKinematics
 * 
 * @param mode 
 * if mode=0    calculcate foot position(legCmdPos) with jointCmdPos(target),
 * if mode=1    calculcate foot position(legPresPos) with jointPresPos(present) and update legPos_last
 */
void CGebot::ForwardKinematics(int mode)
{
    if(mode==0)
        for(int legNum=0;legNum<4;legNum++)
            mfLegCmdPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();    // need SetJointPos(jointCmdPos)
    else{
        mfLegLastPos=mfLegPresPos;
        for(int legNum=0;legNum<4;legNum++)
            mfLegPresPos.row(legNum)=m_glLeg[legNum]->ForwardKinematic().transpose();  // need SetJointPos(jointPresPos)
    }
}

void CGebot::InverseKinematics(Matrix<float, 4, 3> cmdpos)
{
    for(int legNum=0;legNum<4;legNum++)
        mfJointCmdPos.row(legNum)=m_glLeg[legNum]->InverseKinematic(cmdpos.block(legNum,0,1,3)).transpose();
}

//motor control;
 



//robot's air control 
//RF-RH-LH-LF
void CGebot::PumpAllNegtive()
{
    svStatus=0b01010101;// 01-N 10-P;
    api.setSV(svStatus);
}
void CGebot::PumpAllPositve()
{
    svStatus=0b10101010;
    api.setSV(svStatus);
}
void CGebot::PumpPositive(int legNum)
{
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2+1);
    //cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
    api.setSV(svStatus);
}
void CGebot::PumpNegtive(int legNum)
{   
    if(legNum==0) legNum=3;
    else if(legNum==1) legNum=0;
    else if(legNum==2) legNum=2;
    else if(legNum==3) legNum=1;
    svStatus|=1<<((3-legNum)*2);
   //  cout<<"svStatus1="<<bitset<8>(svStatus)<<"\n";
    svStatus&=~(1<<((3-legNum)*2+1));
    //cout<<"svStatus2="<<bitset<8>(svStatus)<<"\n";
    api.setSV(svStatus);
}
 /**
 * @brief control SV to ensure that robot has a suitable positive and negative pressure state to adhere to the wall
 * 
 */
void CGebot::AirControl()
{
    for(int legNum=0;legNum<4;legNum++)
    {
        if(m_glLeg[legNum]->GetLegStatus()==swingDown)  //attach
        {
            PumpNegtive(legNum);
        }
        else if(m_glLeg[legNum]->GetLegStatus()==stance)// Apply negative pressure in advance to solve gas path delay.
        {
            if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
            {
                PumpPositive(legNum);
                BSwingPhaseStartFlag = true;
            }    
        }
        else if(m_glLeg[legNum]->GetLegStatus()==detach)
            PumpPositive(legNum);
        //cout<<"svStatus:"<<std::setw(2)<<svStatus<<endl;
    }
}
/**
 * @brief Correct body tilt.
 * Notice: Applicable to amble gait in 180 degree
 */
void CGebot::AttitudeCorrection180()
{
    int legNum;
    if(BSwingPhaseStartFlag == true )
    {
        BSwingPhaseStartFlag = 0;
        for(legNum=0;legNum<4;legNum++)
        {
            switch(m_glLeg[legNum]->GetLegStatus()) // select swing leg
            {
            case detach:
            case swingUp:
            case swingDown:
            case attach:
                switch (legNum) //the number of swing leg is only 1 in amble gait   
                {
                case 0:
                    mfCompensation(0,2) = 0;
                    mfCompensation(1,2) = CompDisA3;
                    mfCompensation(2,2) = CompDisA2;
                    mfCompensation(3,2) = CompDisA1;
                    break;
                case 1:
                    mfCompensation(0,2) = CompDisA3;
                    mfCompensation(1,2) = 0;
                    mfCompensation(2,2) = CompDisA1;
                    mfCompensation(3,2) = CompDisA2;
                    break;
                case 2:
                    mfCompensation(0,2) = CompDisA2;
                    mfCompensation(1,2) = CompDisA1;
                    mfCompensation(2,2) = 0;
                    mfCompensation(3,2) = CompDisA3;
                    break;
                case 3:
                    mfCompensation(0,2) = CompDisA1;
                    mfCompensation(1,2) = CompDisA2;
                    mfCompensation(2,2) = CompDisA3;
                    mfCompensation(3,2) = 0;
                    break;
                }
                break;
             case stance:
                if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
                    switch (legNum) //the number of swing leg is only 1 in amble gait   
                    {
                    case 0:
                        mfCompensation(0,2) = 0;
                        mfCompensation(1,2) = CompDisA3;
                        mfCompensation(2,2) = CompDisA2;
                        mfCompensation(3,2) = CompDisA1;
                        break;
                    case 1:
                        mfCompensation(0,2) = CompDisA3;
                        mfCompensation(1,2) = 0;
                        mfCompensation(2,2) = CompDisA1;
                        mfCompensation(3,2) = CompDisA2;
                        break;
                    case 2:
                        mfCompensation(0,2) = CompDisA2;
                        mfCompensation(1,2) = CompDisA1;
                        mfCompensation(2,2) = 0;
                        mfCompensation(3,2) = CompDisA3;
                        break;
                    case 3:
                        mfCompensation(0,2) = CompDisA1;
                        mfCompensation(1,2) = CompDisA2;
                        mfCompensation(2,2) = CompDisA3;
                        mfCompensation(3,2) = 0;
                        break;
                    }
                break;
            }
        }
    }
    if(BSwingPhaseEndFlag == true )
    {
        BSwingPhaseEndFlag = 0;
        mfCompensation(0,2) = CompDisALL;
        mfCompensation(1,2) = CompDisALL;
        mfCompensation(2,2) = CompDisALL;
        mfCompensation(3,2) = CompDisALL;
    }

}

void CGebot::AttitudeCorrection90()
{
    int legNum;
    // float attX=-4, attZ=6, attALLx=-3;           //p
    float attX=-4*2.5/1000, attZ=6*3/1000, attALLx=-3*2.5/1000;  //adm
    if(BSwingPhaseStartFlag == true )
    {
        BSwingPhaseStartFlag = 0;
        for(legNum=0;legNum<4;legNum++)
        {
            switch(m_glLeg[legNum]->GetLegStatus()) // select swing leg
            {
            case detach:
            case swingUp:
            case swingDown:
            case attach:
                switch (legNum) //the number of swing leg is only 1 in amble gait   
                {
                case 0:
                    mfCompensation<< 0, 0, 0, 
                                    attX, 0, attZ, 
                                    attX, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 1:
                    mfCompensation<< attX, 0, attZ, 
                                     0, 0, 0, 
                                    attX, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 2:
                    mfCompensation<<attX, 0, 0, 
                                    attX, 0, 0, 
                                     0, 0, 0, 
                                    attX, 0, 0;
                    break;
                case 3:
                    mfCompensation<<attX, 0, 0, 
                                    attX, 0, 0, 
                                    attX, 0, 0, 
                                     0, 0, 0;
                    break;
                }
                break;
             case stance:
                if(iStatusCounter[legNum] <= ceil(iStatusCounterBuffer[legNum][int(stance)] * PrePsotiveFactor) )   
                    switch (legNum) //the number of swing leg is only 1 in amble gait   
                    {
                    case 0:
                        mfCompensation<< 0, 0, 0, 
                                        attX, 0, attZ, 
                                        attX, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 1:
                        mfCompensation<< attX, 0, attZ, 
                                         0, 0, 0, 
                                        attX, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 2:
                        mfCompensation<<attX, 0, 0, 
                                        attX, 0, 0, 
                                         0, 0, 0, 
                                        attX, 0, 0;
                        break;
                    case 3:
                        mfCompensation<<attX, 0, 0, 
                                        attX, 0, 0, 
                                        attX, 0, 0, 
                                         0, 0, 0;
                        break;
                    }
                break;
            }
        }
    }
    if(BSwingPhaseEndFlag == true )
    {
        BSwingPhaseEndFlag = 0;
        mfCompensation<<attALLx, 0, attZ/2.0, 
                        attALLx, 0, attZ/2.0, 
                        attALLx, 0, 0, 
                        attALLx, 0, 0;
    }

}
void CGebot::UpdateTouchStatus(vector<int> values,vector<int> prevalues,vector<int> preprevalues){
    for(int i=0;i<4;i++){
    if(preprevalues[i]>m_threhold[i])
        if(prevalues[i]>m_threhold[i])
            m_glLeg[i]->setTouchStatus(values[i]>m_threhold[i]);
    }
}

MatrixXd load_matrix(const string& filename, int rows, int cols) {
    MatrixXd mat(rows, cols); //从文件中读取矩阵数据。
    ifstream file(filename);//从文件中读取向量数据。

    if (file.is_open()) {
        for (int i = 0; i < rows; ++i) {
            for (int j = 0; j < cols; ++j) {
                file >> mat(i, j);
            }
        }
        file.close();
    } else {
        cout << "Unable to open file";
    }
    return mat;
} //从指定文件中读取矩阵数据，并将其存储在MatrixXd类型的变量中。

VectorXd load_vector(const string& filename, int size) {
    VectorXd vec(size);
    ifstream file(filename);

    if (file.is_open()) {
        for (int i = 0; i < size; ++i) {
            file >> vec(i);
        }
        file.close();
    } else {
        cout << "Unable to open file";
    }
    return vec;
}//从指定文件中读取向量数据，并将其存储在VectorXd类型的变量中。

MatrixXd relu(const MatrixXd& x) {
    return x.cwiseMax(0.0);
}//实现ReLU激活函数，即将输入矩阵中的负值设为0

Matrix<float, 3, 1> CGebot::FnnOutputcpt(VectorXd vec)
{
    VectorXd output(3);
    VectorXd input(6);
    for(int i=0; i<6; i++) input(i) = vec(i);
    int inputNum = 6;
    int layer1Num = 8;
    int outputNum = 3; //定义输入、隐藏层和输出的大小。
    MatrixXd W1 = load_matrix("../fnn_model/fc1.weight_weight.txt", layer1Num, inputNum);
    VectorXd b1 = load_vector("../fnn_model/fc1.bias_weight.txt", layer1Num);
    MatrixXd W2 = load_matrix("../fnn_model/fc2.weight_weight.txt", outputNum, layer1Num);
    VectorXd b2 = load_vector("../fnn_model/fc2.bias_weight.txt", outputNum);//从文件中加载权重和偏置。

    // Example input
    // Vector<float, 6> input;
    // for (int i = 0; i < 6; ++i) 
    // {
    //     input(i) = (*vec)[i];
    // }

    // Forward pass
    MatrixXd layer1 = relu((vec.transpose() * W1.transpose()).transpose() + b1);//进行第一层的计算并应用ReLU激活函数。
    output = (layer1.transpose() * W2.transpose()).transpose() + b2;//计算输出层的结果
    Matrix<float, 3, 1> outputMatrix;
    for(int i=0; i<3; i++) outputMatrix(i,0) = output(i);
    return outputMatrix;
}

Matrix<float,4,3> CGebot::FnnStepModify()
{
    
        //开始编写炳诚师兄布置的任务
        // rbt.NextStep();
        // cout<<"rbt.mfLegCmdPos:"<<rbt.mfLegCmdPos<<endl;
       
        mfLegCmdCompPos = mfLegCmdPos + mfShoulderPosCompensation;
  
        // if(0==swing)  123;
        // if(1) 032
        // if(2) 301
        //if(3) 210
        Matrix<float, 6, 1> input(6);
        int swinglegnum = 0;
        bool foundFlag = 0;
        for(uint8_t legNum=0; legNum<4; legNum++) //遍寻4条腿
        {
        // UpdateLegStatus(legNum); //更新腿的状态
        enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus(); //get present status
            if(ls!=stance&&ls!=recover)
            {
                if(ls!=stance)
                {
                    swinglegnum = legNum;
                }
            }
        
        for(uint8_t legNum=0; legNum<4; legNum++)
        {
            enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus();
            if(ls==detach)
            {
                foundFlag = 1;
                swinglegnum = legNum;
                break;
            }
        }
        if(foundFlag == 0)
        {
            for(uint8_t legNum=0; legNum<4; legNum++)
            {
                enum_LEGSTATUS ls=m_glLeg[legNum]->GetLegStatus();
                if(ls!=stance)
                {
                    foundFlag = 1;
                    swinglegnum = legNum;
                    break;
                }
            }
        }		//已理解
            Matrix<float, 6, 1> colVec;
            //Matrix<float, 6, 1> colVec1;
            //Matrix<float, 6, 1> colVec2;
            //Matrix<float, 6, 1> colVec3;
            Matrix<float, 3, 1> output(3);
            VectorXd inputxd(6);
            // VectorXd colVec0(6);
            // VectorXd colVec1(6);
            // VectorXd colVec2(6);
            // VectorXd colVec3(6);
            switch(swinglegnum)
            {
                case 0:
                    // 创建一个 6x1 的列向量，用于存储提取的元素
                
                    colVec.segment<1>(0) = mfLegCmdCompPos.block<1, 1>(1, 0).transpose(); // 提取第二行的第一列
                    colVec.segment<1>(1) = mfLegCmdCompPos.block<1, 1>(2, 0).transpose(); // 提取第三行的第二列
                    colVec.segment<1>(2) = mfLegCmdCompPos.block<1, 1>(3, 0).transpose(); // 提取第四行的前第三列
                    colVec.segment<1>(3) = mfLegCmdCompPos.block<1, 1>(1, 1).transpose(); // 提取第二行的前第一列
                    colVec.segment<1>(4) = mfLegCmdCompPos.block<1, 1>(2, 1).transpose(); // 提取第三行的前第二列
                    colVec.segment<1>(5) = mfLegCmdCompPos.block<1, 1>(3, 1).transpose(); // 提取第四行的前第三列
                    input = colVec;
                    for(int i=0; i<6; i++) inputxd(i) = input(i);
                    output = FnnOutputcpt(inputxd);
                    //--needed to be checked by Weilong--  ok
                    mfLegCmdCompPos(1,2) = output(0);
                    mfLegCmdCompPos(2,2) = output(1);
                    mfLegCmdCompPos(3,2) = output(2);
                    break;
                case 1:
                    // 创建一个 6x1 的列向量，用于存储提取的元素
                    
                    colVec.segment<1>(0) = mfLegCmdCompPos.block<1, 1>(0, 0).transpose(); // 提取第一行的第一列
                    colVec.segment<1>(1) = mfLegCmdCompPos.block<1, 1>(3, 0).transpose(); // 提取第四行的第一列
                    colVec.segment<1>(2) = mfLegCmdCompPos.block<1, 1>(2, 0).transpose(); // 提取第三行的第一列
                    colVec.segment<1>(3) = mfLegCmdCompPos.block<1, 1>(0, 1).transpose(); // 提取第一行的第二列
                    colVec.segment<1>(4) = mfLegCmdCompPos.block<1, 1>(3, 1).transpose(); // 提取第四行的第二列
                    colVec.segment<1>(5) = mfLegCmdCompPos.block<1, 1>(2, 1).transpose(); // 提取第三行的第二列
                    input = colVec;
                    for(int i=0; i<6; i++) inputxd(i) = input(i);
                    output = FnnOutputcpt(inputxd);
                    //--needed to be checked by Weilong--  ok
                    mfLegCmdCompPos(0,2) = output(0);
                    mfLegCmdCompPos(3,2) = output(1);
                    mfLegCmdCompPos(2,2) = output(2);
                    break;
                case 2:
                    // 创建一个 6x1 的列向量，用于存储提取的元素
                    
                    colVec.segment<1>(0) = mfLegCmdCompPos.block<1, 1>(3, 0).transpose(); // 提取第四行的第一列
                    colVec.segment<1>(1) = mfLegCmdCompPos.block<1, 1>(0, 0).transpose(); // 提取第一行的第一列
                    colVec.segment<1>(2) = mfLegCmdCompPos.block<1, 1>(1, 0).transpose(); // 提取第二行的第一列
                    colVec.segment<1>(3) = mfLegCmdCompPos.block<1, 1>(3, 1).transpose(); // 提取第四行的第二列
                    colVec.segment<1>(4) = mfLegCmdCompPos.block<1, 1>(0, 1).transpose(); // 提取第一行的第二列
                    colVec.segment<1>(5) = mfLegCmdCompPos.block<1, 1>(1, 1).transpose(); // 提取第二行的第二列
                    input = colVec;
                    for(int i=0; i<6; i++) inputxd(i) = input(i);
                    output = FnnOutputcpt(inputxd);
                    //--needed to be checked by Weilong--	ok
                    mfLegCmdCompPos(3,2) = output(0);
                    mfLegCmdCompPos(0,2) = output(1);
                    mfLegCmdCompPos(1,2) = output(2);
                    break;
                case 3:
                    // 创建一个 6x1 的列向量，用于存储提取的元素
                	colVec.segment<1>(0) = mfLegCmdCompPos.block<1, 1>(2, 0).transpose(); // 提取第三行的第一列
                    colVec.segment<1>(1) = mfLegCmdCompPos.block<1, 1>(1, 0).transpose(); // 提取第二行的第一列
                    colVec.segment<1>(2) = mfLegCmdCompPos.block<1, 1>(0, 0).transpose(); // 提取第一行的第一列
                    colVec.segment<1>(3) = mfLegCmdCompPos.block<1, 1>(2, 1).transpose(); // 提取第三行的第二列
                    colVec.segment<1>(4) = mfLegCmdCompPos.block<1, 1>(1, 1).transpose(); // 提取第二行的第二列
                    colVec.segment<1>(5) = mfLegCmdCompPos.block<1, 1>(0, 1).transpose(); // 提取第一行的第二列
                    input = colVec;
                    for(int i=0; i<6; i++) inputxd(i) = input(i);
                    output = FnnOutputcpt(inputxd);
                    //--needed to be checked by Weilong--	ok
                    mfLegCmdCompPos(2,2) = output(0);
                    mfLegCmdCompPos(1,2) = output(1);
                    mfLegCmdCompPos(0,2) = output(2);
                    
                    break;
            }
        }
        // cout<< "mfLegCmdPos:"<< mfLegCmdPos<<endl;        
        cout<< "mfLegCmdCompPos:"<< mfLegCmdCompPos<<endl;
        // mfCompensationZ = output;
        return mfLegCmdCompPos;
        //结束   
        
}
