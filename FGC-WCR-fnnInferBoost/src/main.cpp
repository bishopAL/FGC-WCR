#include"robotcontrol.h"
#include "ADS1x15.h"
// #include "fnn_stepgenerator.h"

std::vector<int> ID;
#define CHECK_RET(q) if((q)==false){return 0;}
CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
DxlAPI motors("/dev/ttyAMA0", 1000000, ID, 2);
bool runFlag=0;
float torque[12];

void *udpConnect(void *data)// 线程1
{

	string ip = "127.0.0.1";
	uint16_t port = 8888;

	CUdpSocket srv_sock;
	//创建套接字
	CHECK_RET(srv_sock.Socket());
	//绑定地址信息
	CHECK_RET(srv_sock.Bind(ip, port));
	while(1)
	{
		//接收数据
		string buf; //用于存储接收到的数据。
		string peer_ip;
		uint16_t peer_port;//用于存储发送方（上位机）的IP地址和端口号。
        // uint8_t swingflag; 
		CHECK_RET(srv_sock.Recv(&buf, &peer_ip, &peer_port));//接收来自上位机的数据，存储在 buf 中，并获取发送方的IP地址和端口号。
		cout << "UpperComputer["<<peer_ip<<":"<<peer_port<<"] Command: " << buf << endl;//输出接收到的命令及其来源。
        //buf match command 
        
        //z补偿跟新
        // fnn_stepmodify();

        int ret=commandJudge((char*)string("start").c_str(),(char *)buf.c_str());//检查接收到的命令 buf 是否为 "start"。
        if(ret) {runFlag=1; goto END;}//如果匹配，设置 runFlag = 1 并跳转到 END 标签，终止本次循环。
        ret=commandJudge((char*)string("stop").c_str(),(char *)buf.c_str());//检查接收到的命令 buf 是否为 "stop"。
        if(ret) {runFlag=0; goto END;}//如果匹配，设置 runFlag = 0 并跳转到 END 标签，终止本次循环。
        // int ret=match((char*)string("start").c_str(),(char*)string("startsada").c_str());
        // cout<<(char*)string("start").c_str()<<endl;
        // cout<<ret<<endl;
		//发送数据
        END:
		buf.clear();//清空 buf 以便下一次循环使用。
		// cout << "server say: ";
		// cin >> buf;
		// CHECK_RET(srv_sock.Send(buf, peer_ip, peer_port));
	}
	//关闭套接字
	srv_sock.Close();//关闭UDP套接字，释放资源。
	return 0;
}

void *dataSave(void *data) // 线程4
{
    struct timeval startTime,endTime;
    double timeUse;
    ofstream data_IMU, data_Force, data_Torque;//ofstream 对象：data_IMU, data_Force, data_Torque 是文件流对象，用于将数据写入不同的CSV文件。
    float fAngleZero[3], fDataForce[12], fDataTorque[12];//用于存储初始的IMU角度值,fDataForce[12] 和 fDataTorque[12]：分别存储12个力传感器的数据和12个力矩传感器的数据。
    int status[4];//用于存储四个状态值。
    string add="../include/data_IMU.csv";
    //data_IMU.open(add, ios::app); // All outputs are attached to the end of the file.
    data_IMU.open(add);   // cover the old file  以覆盖模式打开IMU数据文件。如果文件已存在，则覆盖该文件，否则创建新文件。
    if (data_IMU)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    data_IMU<<"Angle_pitch_roll_yaw:"<<endl;//在文件中写入表头，以表示上述数据是角度数据。
    usleep(1e3);//暂停1毫秒，以确保文件操作稳定。

    add="../include/data_Force.csv";
    data_Force.open(add);   // cover the old file  打开力传感器数据文件
    if (data_Force)    cout<<add<<" file open Successful"<<endl;
    else    cout<<add<<" file open FAIL"<<endl;
    data_Force<<"Force_x0y0z0_x1y1z1_..._status0123:"<<endl;//在文件中写入表头，以表示上述数据是Force_x0y0z0_x1y1z1_..._status0123
    usleep(1e3);

    // add="../include/data_Torque.csv";
    // data_Torque.open(add);   // cover the old file
    // if (data_Torque)    cout<<add<<" file open Successful"<<endl;
    // else    cout<<add<<" file open FAIL"<<endl;
    // data_Torque<<"Torque_0-12:"<<endl;
    // usleep(1e3);

    while(rbt.bInitFlag == 0) //wait for initial  在开始数据采集之前，线程会等待 rbt.bInitFlag 标志被设置为1。这通常意味着系统的初始化（例如硬件准备）已经完成。
        usleep(1e2);
     rbt.UpdateImuData(); 
    for (int i = 0; i < 3; i++)
        fAngleZero[i] = rbt.api.fAngle[i];//这三行用于读取并存储IMU的初始角度值，以便在后续计算中进行角度补偿。 还需要再查下
    // WitCaliRefAngle();                               //  归零失败
    // u16 xx = rbt.api.fAngle[0] * 32768.0f / 180.0f;  
    // WitWriteReg(XREFROLL, xx); //sReg[Roll]          //  归零失败
	while(1)
	{
        if(runFlag)
        {
            gettimeofday(&startTime,NULL);
            //record data数据采集       Prevent simultaneous access to the same memory!
             rbt.UpdateImuData();//更新IMU数据
            for (int i = 0; i < 3; i++)
                for (int j = 0; j < 4; j++)
                    fDataForce[i*4+j]=rbt.mfForce(i, j);  //数据保存
            // for (int i = 0; i < 12; i++)
            //     fDataTorque[i]=rbt.dxlMotors.present_torque[i];
            // for (size_t i = 0; i < 4; i++)
            //     status[i]=rbt.m_glLeg[i]->GetLegStatus();
            //write data
            for (int i = 0; i < 3; i++)//遍历了IMU传感器的三个角度（通常是俯仰角、横滚角和偏航角），这些角度保存在 rbt.api.fAngle[i] 数组中。
            {
                data_IMU<<rbt.api.fAngle[i]-fAngleZero[i]<<",";  //计算当前角度与初始角度（即程序开始时记录的角度）的差值。这种补偿有助于减去初始偏差，只记录相对于初始状态的角度变化。
                // cout<<"angle_"<<i<<": "<<rbt.api.fAngle[i]-fAngleZero[i]<<endl;
            }
            data_IMU<<endl;//在每一行的末尾加上换行符，以将下一组数据写入新行。
            // for (int i = 0; i < 3; i++)
            //     for (int j = 0; j < 4; j++)
            //         data_Force<<rbt.mfForce(i, j)<<",";  
            // data_Force<<endl;
            // for (int i = 0; i < 12; i++)
            //     data_Torque<<rbt.dxlMotors.present_torque[i]<<",";
            //  data_Torque<<endl;

            for (size_t i = 0; i < 12; i++)//遍历了力传感器的12个数据点，这些数据点保存在 fDataForce[i] 数组中。
            {
                data_Force<<fDataForce[i]<<",";//将力传感器数据写入 data_Force 文件，并用逗号分隔不同的数据值。
                // data_Torque<<fDataTorque[i]<<",";
                // data_Torque<<torque[i]<<",";
            }
            // for (size_t i = 0; i < 4; i++)
            //      data_Force<<status[i]<<",";
            data_Force<<endl;//在每一行的末尾加上换行符，以将下一组数据写入新行。
            // data_Torque<<endl;
                // for (size_t i = 0; i < 12; i++)
                // {
                //     cout<<torque[i]<<",";
                // }
                // cout<<endl<<endl;

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e5)
                usleep(1.0/loopRateDataSave*1e6 - (double)(timeUse) - 10); 
            else
                cout<<"dataSave: "<<timeUse<<endl;
        }
	}
    data_IMU.close();data_IMU.clear();
    data_Force.close();data_Force.clear();//关闭打开的文件并清空文件流对象。
    // data_Torque.close();data_Torque.clear();
}


void *robotStateUpdateSend(void *data) //线程2
{
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV;
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha 
    
    
    //motors initial

#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            rbt.mfJointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //rbt.forwardKinematics
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    rbt.forwardKinematics(0);
    rbt.setInitPos(rbt.mfLegCmdPos);        //legCmdPos
    cout<<"legCmdPos:\n"<<rbt.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);
#endif    
 
    //      rbt initial
    // TimeForStancePhase<< 0,                       TimeForGaitPeriod/2.0,     // diagonal
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      TimeForGaitPeriod/2.0,   TimeForGaitPeriod, 
    //                      0,                       TimeForGaitPeriod/2.0;
    // TimeForSwingPhase<< TimeForGaitPeriod/4.0 *2,          TimeForGaitPeriod/4.0 *3,   // tripod
    //                      0,             TimeForGaitPeriod/4.0,
    //                      TimeForGaitPeriod/4.0 *3,    TimeForGaitPeriod,
    //                      TimeForGaitPeriod/4.0  ,          TimeForGaitPeriod/4.0 *2;
    TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
                        0,		 		 					3*TimeForGaitPeriod/16,		
                        12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);

#if(INIMODE==2)
   float  float_initPos[12]={   70.0,65.5,-21.0,
                                70.0,-65.5,-21.0,
                               -84.0, 65.5,-21.0,
                               -84.0, -65.5,-21.0};
// 60, 60, -30,
// 60,-60, -30,
// -60, 60, -30,
// -60,-60, -30,
// 65.5,70.0,21.0,
// 65.5,70.0,21.0,
// 65.5,84.0,21.0,
// 65.5,84.0,21.0
//  80,  50, -22,
//  80, -50, -22,
// -40,  50, -22,
// -40, -50, -23,

//  80,  55, -16,
//  80, -55, -16,
// -40,  55, -16,
// -40, -69, -18,
//    float  float_initPos[12];
//    string2float("../include/initPos.csv", float_initPos);//Foot end position
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            InitPos(i, j) = float_initPos[i*3+j]/1000;
            //cout<<InitPos(i, j)<<endl;
        }
    rbt.SetInitPos(InitPos);
#endif
  
    rbt.InertiaInit();
    rbt.SetCoMVel(TCV);
    rbt.InverseKinematics(rbt.mfLegCmdPos);
    rbt.mfTargetPos = rbt.mfLegCmdPos;
  
#if(INIMODE==2)  
    SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);
    cout<<"rbt.mfJointCmdPos:"<<rbt.mfJointCmdPos<<endl;
    
#endif
    usleep(1e5);
    for (size_t i = 0; i < 4; i++)
        rbt.PumpNegtive(i);
    usleep(1e6);
    rbt.bInitFlag = 1;

    Matrix<float, 4, 3> mfJointTemp; 
    while(1)
    {
        if(runFlag&&rbt.autoControlFlag)
        {
            std::ofstream outFile;
            std::ofstream outFile1;

            outFile.open("../include/compensationrecord.csv");
            if (!outFile) 
            {
            std::cerr << "无法打开文件进行写入" << std::endl;
            }

            outFile1.open("../include/rawrecord.csv");

            if (!outFile1) 
            {
            std::cerr << "无法打开文件进行写入" << std::endl;
            }

            for(int times=0; times<2000; times++)
            {
                struct timeval startTime,endTime;
                double timeUse;
                gettimeofday(&startTime,NULL);
                //If stay static, annotate below one line.
                std::cout<<std::endl;
                std::cout<<"times"<<times<<std::endl;

                rbt.NextStep();
                // cout<<"rbt.mfCompensationZ:"<<rbt.mfCompensationZ<<endl;
                Matrix<float,4,3> mfLegCmdCompPos1=rbt.FnnStepModify();
                cout<<"mfLegCmdCompPos1"<<mfLegCmdCompPos1<<endl;
                rbt.InverseKinematics(mfLegCmdCompPos1);

                //将数据结果保存到文件里面
                outFile << mfLegCmdCompPos1(0,2)  << " "<<endl;	
                outFile1 << rbt.mfLegCmdPos(0,2)  << " "<<endl;	

                rbt.AirControl();
                // rbt.AttitudeCorrection90();
                
                // rbt.ParaDeliver();
                
                // rbt.UpdateImuData();     // segmentation fault
                //cout<<"LegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;    
                // cout<<"TargetPos:\n"<<rbt.mfTargetPos<<endl<<endl; 
                // cout<<"Compensation:\n"<<rbt.mfCompensation<<endl<<endl; 
                rbt.UpdatejointPresPosAndVel(motorMapping(rbt.mfJointCmdPos));
                rbt.ForwardKinematics(1);

                gettimeofday(&endTime,NULL);
                timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
                if(timeUse < 1e4)
                    usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 10); 
                else
                cout<<"TimeRobotStateUpdateSend: "<<timeUse<<endl;
            }
            outFile.close();
            outFile1.close();
        }
    }
 
}

void *runImpCtller(void *data) //线程3
{
    struct timeval startTime,endTime;
    double timeUse;
    int run_times=0;    // for debugging

    while(rbt.bInitFlag == 0) //wait for initial
        usleep(1e2);

    //rbt.dxlMotors.torqueEnable();
    while (1)
    {
        if(runFlag&&rbt.autoControlFlag)
        {
            gettimeofday(&startTime,NULL);
            /* get motors data  */
            motors.getTorque();
            motors.getPosition();
            motors.getVelocity();

            /* update the data IMP need */
            rbt.UpdatejointPresPosAndVel(motors.present_position);         
            //rbt.UpdatejointPresVel(); //useless,wrong data
            rbt.ForwardKinematics(1);
            rbt.UpdateJacobians();
           // rbt.UpdateFtsPresVel();
            rbt.UpdateFtsPresForce(motors.present_torque);  
            // for (size_t i = 0; i < 12; i++)
            // {
            //     torque[i] = rbt.dxlMotors.present_torque[i];
            // }            


            /*      Admittance control     */ 
            // rbt.Control();   
            // rbt.InverseKinematics(rbt.mfXc);    // Admittance control

            /*      Postion control with Comp      */
            // rbt.InverseKinematics(rbt.mfTargetPos); //    Postion control

            /*      Postion control      */
             rbt.InverseKinematics(rbt.mfLegCmdPos); 
             cout<<endl;
             cout<<"mfLegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;
             cout<<"mfJointCmdPos:\n"<<rbt.mfJointCmdPos<<endl;
            // cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<endl;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            //cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;

            /*      Set joint angle      */
            SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);

            /*      Impedance control      */
            // for(int i=0; i<4; i++)  
            //     for(int j=0;j<3;j++)
            //         SetTorque[i*3+j] = rbt.target_torque(j,i);
            // motors.setTorque(SetTorque); 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); 
            // else
            //     cout<<"timeImpCtller: "<<timeUse<<endl;
        }
    }
  
}
void *SvUpdate(void *data)//线程5  用于更新传感器数据，并根据传感器的状态调整 rbt 对象的行为。
{   
    ADS1015 ads; //创建一个 ADS1015 对象 ads，用于与ADC（模拟-数字转换器）进行交互
    vector<int> value(4),preValue(4),prepreValue(4);//创建三个 vector 对象，分别用于存储当前的传感器数据、前一时刻的传感器数据和前前时刻的传感器数据。每个 vector 都有4个整数元素。
    for(auto a:value)
        a=0; //这行代码应该将 value 中的所有元素初始化为0。然而，使用 for (auto a : value) 并不会改变 value 中的元素，应该使用 for (int &a : value) 以便修改元素值。
    preValue=value;
    prepreValue=value;//将 preValue 和 prepreValue 初始化为 value 的当前值。这样可以在首次迭代中有一个有效的初始值。
    while(1){
        struct timeval startTime,endTime;
        double timeUse;
        int gain=1;//设置ADC的增益为1
        for(int i=0;i<4;i++)
        {
            value[i]=(int)ads.read_adc(i,gain); //调用 ads.read_adc 方法读取指定通道的ADC值，并将其存储在 value 向量中。
            usleep(10000);//在读取每个通道数据后，暂停10毫秒（10000微秒），以避免过于频繁地读取数据。
        }
        rbt.UpdateTouchStatus(value,preValue,prepreValue); //调用 UpdateTouchStatus 方法来更新 rbt 对象的触摸状态。这通常涉及到比较当前值 value 与先前值 preValue 和 prepreValue，以检测触摸状态的变化。
        prepreValue=preValue;
        preValue=value;//更新历史数据，将 preValue 设为当前的 value，将 prepreValue 设为先前的 preValue。这样在下一次循环时，就有新的历史数据用于比较。

        if(rbt.autoControlFlag == false){//检查是否需要进行自动控制。
            int count=0; //用于计数有多少个腿部传感器的触摸状态为真。
            for(auto a:rbt.m_glLeg){//遍历 rbt 对象中的腿部传感器列表，检查每个传感器的触摸状态。
                if(a->getTouchStatus()==true)   count++;//如果某个传感器的触摸状态为真，则 count 增加。
            }
            if(count == 4){
                rbt.autoControlFlag=true;//如果所有四个传感器的触摸状态均为真，则设置 rbt.autoControlFlag 为真。
                for (size_t i = 0; i < 4; i++)//遍历所有腿部传感器，如果某个传感器的状态不是 recover 且 stance，则触发 rbt.probeTrigger[i]。
                {
                    if(rbt.m_glLeg[i]->GetLegStatus()!=recover&&rbt.m_glLeg[i]->GetLegStatus()!=stance)
                      rbt.probeTrigger[i]=true;
                }
            }
        }

        gettimeofday(&endTime,NULL);
        timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
        if(timeUse < 1e4)
        usleep(1.0/loopRateSVRead*1e6 - (double)(timeUse) - 10); 
    }

}
int main(int argc, char ** argv)
{   
    pthread_t th1, th2, th3, th4,th5;
	int ret;
    ret = pthread_create(&th1,NULL,udpConnect,NULL);
    if(ret != 0)
	{
		printf("create pthread1 error!\n");
		exit(1);
	}
    ret = pthread_create(&th2,NULL,robotStateUpdateSend,NULL);
    if(ret != 0)
	{
		printf("create pthread2 error!\n");
		exit(1);
	}
    ret = pthread_create(&th3,NULL,runImpCtller,NULL);
    if(ret != 0)
	{
		printf("create pthread3 error!\n");
		exit(1);
	}
    ret = pthread_create(&th4,NULL,dataSave,NULL);
    if(ret != 0)
	{
		printf("create pthread4 error!\n");
		exit(1);
	}
    //  ret = pthread_create(&th5,NULL,SvUpdate,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread4 error!\n");
	// 	exit(1);
	// }
    
    pthread_join(th1, NULL);
    pthread_join(th2, NULL);// zuichu
    pthread_join(th3, NULL);
    pthread_join(th4, NULL);
    //pthread_join(th5, NULL);
    while(1);

    
    
    return 0;
}


// //bendi test
// // #include "i2c.h"
// // #include "api.h"
// // #include <stdio.h>
// // #include <unistd.h>
// // #include <wiringPi.h>
// // #include "dynamixel.h"
// // #include <vector>
// // #include <time.h>
// // #include <stdlib.h>
// #include "robotcontrol.h"
// #include "ADS1x15.h"
// #define loopRate 100 //hz

// #define LF_PIN      1
// #define RF_PIN      24
// #define LH_PIN      28
// #define RH_PIN      29
// uint8_t svStatus=0b01010101;
// // uint8_t svStatus=0b10101010;


// int main()
// {
      
//     /*********adc test********/
//     struct timeval startTime,endTime;
//     double timeUse;
//    // API api;
//     vector<int> ID;
//     vector<float> start_pos;
//     vector<float> target_tor;

//     // for(int i=0; i<4; i++)
//     // {
//     // ID.push_back(3*i);
//     // ID.push_back(3*i+1);
//     // start_pos.push_back(0.00);
//     // start_pos.push_back(0.00);
//     // }
//      for(int i=0; i<16; i++)
//     {
//     ID.push_back(i);
//     start_pos.push_back(0.00);
//     }//把id和初始位置设置下，分别放到向量容器中
//      DxlAPI gecko("/dev/ttyAMA0", 1000000, ID, 2); //ttyUSB0
// //     // gecko.setBaudRate(5);
//      gecko.setOperatingMode(3);  //3 position control; 0 current control
//      gecko.torqueEnable();
//      gecko.setPosition(start_pos);
//    // gecko.getPosition();
// //     int times=1;
// //     bool reverse=false;
// //     while(1)
// //     {
// //             gecko.getTorque();
// //             gecko.getPosition();
// //             gecko.getVelocity();
// //             cout<<" present postion: ";
// //             for(int i=0;i<16;i++)
// //             cout<<gecko.present_position[i]<<" ";
// //             cout<<endl;
// //     }
// //     //~ usleep(1e6);
//     //  api.setPump(1, LOW);
//     //  api.setPump(24, LOW);
//     //  api.setPump(28, LOW);
//     //  api.setPump(29, LOW);
//     // float torque[12];
//     // usleep(1e6);
//     //api.setSV(svStatus);


//     CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
//     Matrix<float,4,2> TimeForSwingPhase;
//     Matrix<float, 4, 3> InitPos;
//     Matrix<float, 6,1> TCV;
//     TCV << 3.0/1000.0, 0, 0,0,0,0 ;// X, Y , alpha 
   
// float  float_initPos[12]={   70.0,65.5,-30.0,
//                              70.0,-65.5,-30.0,
//                             -70.0, 65.5,-30.0,
//                             -70.0, -65.5,-30.0};
//     // float  float_initPos[12];
//     // string2float("../include/initPos.csv", float_initPos);//Foot end position
//     for(int i=0; i<4; i++)
//         for(int j=0;j<3;j++)
//         {
//             InitPos(i, j) = float_initPos[i*3+j]/1000;
//             // cout<<InitPos(i, j)<<endl;
//         }
//     rbt.SetInitPos(InitPos);
//     rbt.InverseKinematics(rbt.mfLegCmdPos);
//     cout<<"cmdPos: "<<rbt.mfJointCmdPos<<endl;
//    // rbt.SetPos(rbt.mfJointCmdPos);
//  //  rbt.PumpAllPositve();
//     usleep(1e6);
//    rbt.SetCoMVel(TCV);
//     TimeForSwingPhase<< 8*TimeForGaitPeriod/16, 	11*TimeForGaitPeriod/16,		
//                         0,		 		 					3*TimeForGaitPeriod/16,		
//                         12*TimeForGaitPeriod/16, 	15*TimeForGaitPeriod/16,		
//                         4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;
//     rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);

//     std::ofstream outFile;
//     // outFile.open("../include/compensationrecord.csv");
//     outFile.open("../include/compensationrecordtxt.txt");
//     // 检查文件是否成功打开
//     if (!outFile) {
//         std::cerr << "无法打开文件进行写入" << std::endl;
//         return 1;
//     }

// //     //ADS1015 ads;

   
// //     // while(1){
// //     //     struct timeval startTime,endTime;
// //     //     double timeUse;
// //     //     gettimeofday(&startTime,NULL);
// //     //     for(int i=0;i<4;i++){
// //     //         value[i]=ads.read_adc(i,gain);
// //     //           usleep(8000);
// //     //     }
// //     //     for(auto a:value){
// //     //         std::cout<<a<<" ";
// //     //     }
// //     //     std::cout<<std::endl;
// //     //     gettimeofday(&endTime,NULL);
// //     //     timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec)+ endTime.tv_usec - startTime.tv_usec;
// //     //     std::cout<<"timeUse="<<timeUse<<std::endl;
// //     // }
// //     /*********adc test********/
// //      vector<int> value(4),preValue(4),prepreValue(4);
// //     for(auto a:value)
// //         a=0;
// //     preValue=value;
// //     prepreValue=value;
//    struct timeval startTimeswing,endTimeswing;
//     double timeUseswing;
//      gettimeofday(&startTimeswing,NULL);
//     for(int times=0; times<2000; times++)
//     {
//         struct timeval startTime,endTime;
//         double timeUse;
//         gettimeofday(&startTime,NULL);
//         std::cout<<std::endl;
//         std::cout<<"times"<<times<<std::endl;
//         // gecko.getTorque();
//         // gecko.getPosition();
//         // gecko.getVelocity();
//         rbt.NextStep();
//         Matrix<float,4,3> mfLegCmdCompPos1=rbt.FnnStepModify();

//         cout<<"mfLegCmdCompPos1"<<mfLegCmdCompPos1<<endl;
// 		//rbt.ParaDeliver();
//         rbt.InverseKinematics(mfLegCmdCompPos1);
//         //for(uint8_t legNum=0; legNum<4; legNum++)  // run all 4 legs
//         //{   
//         //    enum_LEGSTATUS ls=rbt.m_glLeg[legNum]->GetLegStatus(); //get present status
//         //    outFile << ls << " ";
//         //	 cout<<ls<<" ";
//         //}
//         //cout<<endl;
//         //for (int index=0; index<3; index++)
//         //{
//         //    outFile << rbt.mfCompensationZ(index) << " ";
//         //}
//         //outFile << std::endl;
//         //cout<<"mfLegCmdCompPos: \n"<<rbt.mfLegCmdCompPos<<"\n";//<<"mfJointCmdPos: \n"<<rbt.mfJointCmdPos<<endl;
			
//         //重写一个函数将数据保存到outfile里面
//         outFile << mfLegCmdCompPos1(0,2)  << " "<<endl;		
        
//         // cout<<"mfLegCmdPos"<<rbt.mfLegCmdPos<<endl;

//         rbt.UpdatejointPresPosAndVel(motorMapping(rbt.mfJointCmdPos));
//         rbt.ForwardKinematics(1);
//         // cout<<"forward: " <<rbt.mfLegPresPos<<endl;
//         // cout<<" present postion: ";
//         // for(int i=0;i<16;i++)
//         // cout<<gecko.present_position[i]<<" ";
//         // cout<<endl;
//         //SetPos(rbt.mfJointCmdPos,gecko,rbt.vLastSetPos);
//         gettimeofday(&endTime,NULL);
//         timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
//         if(timeUse < 1.0/loopRateStateUpdateSend*1e6)
//         usleep(1.0/loopRateStateUpdateSend*1e6 - (double)(timeUse) - 2000); 

//         // if(swingtimeFlag==true){
//         //     gettimeofday(&endTimeswing,NULL);
//         //     timeUseswing = 1e6*(endTimeswing.tv_sec - startTimeswing.tv_sec) + endTime.tv_usec - startTime.tv_usec;
//         //     cout<<"timeuse="<<timeUseswing<<endl;
//         //     exit(0);
//         //}
//     }
//     outFile.close();
// // //     pos2[11]=((float)times)/2000.0 * 3.14;
// // //     pos2[15]=((float)times)/2000.0 * 3.14;
// // //     pos2[8]=-((float)times)/2000.0 * 3.14;
// // //     pos2[14]=((float)times)/2000.0 * 3.14;
// // //     pos2[5]=-((float)times)/2000.0 * 3.14;
// // //     pos2[13]=((float)times)/2000.0 * 3.14;
// // //     pos2[2]=((float)times)/2000.0 * 3.14;
// // //     pos2[12]=((float)times)/2000.0 * 3.14;
// // //    rbt.dxlMotors.setPosition(pos2);
// // //         // int gain=1;
// // //         // for(int i=0;i<4;i++)
// // //         // {
// // //         //     value[i]=(int)ads.read_adc(i,gain);
// // //         //     usleep(10000);
// // //         // }
// // //         // rbt.UpdateTouchStatus(value,preValue,prepreValue);
// // //         // prepreValue=preValue;
// // //         // preValue=value;

// // //        // rbt.SetPos(rbt.mfJointCmdPos);
// // //         usleep(1e3);
// // //     }
// //     int times=0;
// //    bool reverse=false;
// //     while(1){
        
// //        // std::cout<<times<<std::endl;
// //        // rbt.NextStep();
// //        //rbt.InverseKinematics(rbt.mfLegCmdPos);
// //       rbt.dxlMotors.getTorque();
// //      rbt.dxlMotors.getPosition();
// //      rbt.dxlMotors.getVelocity();
// //     vector<float> pos2(16);
// //     for(auto a : pos2){
// //         a=0;
// //     }
// //     if(!reverse){
// //     times++;
// //     pos2[11]=((float)times)/2000.0 * 3.14;
// //     pos2[15]=((float)times)/2000.0 * 3.14;
// //     pos2[8]=-((float)times)/2000.0 * 3.14;
// //     pos2[14]=((float)times)/2000.0 * 3.14;
// //     pos2[5]=-((float)times)/2000.0 * 3.14;
// //     pos2[13]=((float)times)/2000.0 * 3.14;
// //     pos2[2]=((float)times)/2000.0 * 3.14;
// //     pos2[12]=((float)times)/2000.0 * 3.14;
// //     if(times>1000) reverse=true;
// //     }
// //     else{
// //         times--;
// //     pos2[11]=((float)times)/2000.0 * 3.14;
// //     pos2[15]=((float)times)/2000.0 * 3.14;
// //     pos2[8]=-((float)times)/2000.0 * 3.14;
// //     pos2[14]=((float)times)/2000.0 * 3.14;
// //     pos2[5]=-((float)times)/2000.0 * 3.14;
// //     pos2[13]=((float)times)/2000.0 * 3.14;
// //     pos2[2]=((float)times)/2000.0 * 3.14;
// //     pos2[12]=((float)times)/2000.0 * 3.14;
// //     if(times<0) reverse=false;
// //     }
   
// //    rbt.dxlMotors.setPosition(pos2);   
// //    usleep(1e3); 
// // }
// }