#include"robotcontrol.h"
#include "ADS1x15.h"
// #include "fnn_stepgenerator.h"
#define CHECK_RET(q) if((q)==false){return 0;}
CRobotControl rbt(110.0,60.0,20.0,800.0,ADMITTANCE);
DxlAPI motors("/dev/ttyAMA0", 1000000, rbt.ID, 2);
bool runFlag=1;
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


void *robotStateUpdateSend(void *data) //线程2  它主要负责初始化机器人状态、进行运动学计算和控制周期内的状态更新。
{
    Matrix<float,4,2> TimeForSwingPhase;
    Matrix<float, 4, 3> InitPos;
    Matrix<float, 6,1> TCV; //这些矩阵用于存储步态时间、初始位置和质心速度（CoM Velocity）
    TCV << VELX, 0, 0,0,0,0 ;// X, Y , alpha  表示设定了机器人在 X 方向的初速度为 VELX，其他方向为 0
     
    
    //motors initial
    motors.setOperatingMode(3);
    motors.torqueEnable();
    motors.getPosition();
#if(INIMODE==1)
    vector<float> init_Motor_angle(12);
    float float_init_Motor_angle[12];
    string2float("../include/init_Motor_angle.csv", float_init_Motor_angle);//Motor angle     d  从 CSV 文件 init_Motor_angle.csv 读取初始电机角度
    //cout<<"____________"<<endl;
    for(int i=0; i<4; i++)
        for(int j=0;j<3;j++)
        {
            float_init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j] * 3.1416/180; //to rad 并将这些角度从度数转换为弧度（乘以 π/180）。
            init_Motor_angle[i*3+j] = float_init_Motor_angle[i*3+j];      //vector
            rbt.mfJointCmdPos(i,j) = float_init_Motor_angle[i*3+j];            //rbt.forwardKinematics  初始化的角度赋值给机器人 rbt.mfJointCmdPos 以及电机
            //cout<<init_Motor_angle[i*3+j]<<endl;
        }
    rbt.forwardKinematics(0); //调用 rbt.forwardKinematics(0) 进行正向运动学计算，初始化机器人的状态。
    rbt.setInitPos(rbt.mfLegCmdPos);        //legCmdPos  这一行代码的作用是将当前机器人腿部的目标位置 rbt.mfLegCmdPos 设置为机器人的初始位置。
    cout<<"legCmdPos:\n"<<rbt.legCmdPos<<endl ;

    motors.setPosition(init_Motor_angle);  //这行代码的作用是将电机的目标位置设定为 init_Motor_angle。
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
                        4*TimeForGaitPeriod/16, 	7*TimeForGaitPeriod/16;  //这里设置了四条腿的摆动相位时间，时间基于步态周期 TimeForGaitPeriod。
    rbt.SetPhase(TimePeriod, TimeForGaitPeriod, TimeForSwingPhase);//是一个用于设置机器人步态相位的函数调用，具体的作用是在控制机器人行走过程中定义每个腿的摆动和支撑相位时间。
    //TimePeriod：这是一个总时间周期，代表机器人完成一个完整步态周期（包括摆动相和支撑相）的时间。它可能是秒或毫秒单位。
    //TimeForGaitPeriod：这是机器人步态周期的时间，表示从某一条腿开始摆动到下一次摆动的周期时间。也可以理解为步态的整体周期长度。
    //TimeForSwingPhase：这是一个矩阵或数组，用于定义每条腿的摆动相位时间，通常是针对每条腿什么时候开始摆动以及摆动的持续时间。每一行代表一条腿，每列代表该条腿在不同阶段的摆动开始和结束时间。

#if(INIMODE==2)
//    float  float_initPos[12]={   70.0,65.5,-21.0,
//                                 70.0,-65.5,-21.0,
//                                -84.0, 65.5,-21.0,
//                                -84.0, -65.5,-21.0}; //这里定义了一个大小为 12 的 float 数组 float_initPos，它包含了四条腿的初始末端位置（足端位置），每条腿的末端位置由 3 个数值表示（X, Y, Z 坐标）。因此，四条腿共计 12 个数值。

   float  float_initPos[12]={   60.0,60.0,-30.0,
                                60.0,-60.0,-30.0,
                               -60.0, 60.0,-30.0,
                               -60.0, -60.0,-30.0};
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
            InitPos(i, j) = float_initPos[i*3+j]/1000;  //这里的 for 循环将 float_initPos 数组中的值逐一复制到 InitPos 矩阵中，并将其从毫米单位转换为米单位（因为 /1000 的关系）。
            //cout<<InitPos(i, j)<<endl;
        }
    rbt.SetInitPos(InitPos);  //这行代码将计算好的 InitPos 矩阵传递给机器人对象 rbt，并调用其 SetInitPos 函数，设置机器人的初始足端位置。SetInitPos 可能用于机器人在启动时确定各条腿的初始姿态，以确保机器人从一个已知的稳定状态开始行动。
#endif
  
    rbt.InertiaInit();
    rbt.SetCoMVel(TCV);  //调用 InertiaInit 初始化机器人惯性模型，调用 SetCoMVel 设置质心速度。
    rbt.InverseKinematics(rbt.mfLegCmdPos);
    rbt.mfTargetPos = rbt.mfLegCmdPos;  //使用逆运动学计算初始腿部位置，并将这些位置赋值给目标位置 mfTargetPos。
  
#if(INIMODE==2)  
    motors.setPosition(std::vector<float>(12,0.0));
    //SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);  //这一行代码将 rbt.mfJointCmdPos（机器人的关节目标位置）设置给 motors（电机），并将当前的关节位置 rbt.vLastSetPos 传递给电机以用于后续动作。
    //  for (int i = 0; i < rbt.vLastSetPos.size(); ++i) {
    //     std::cout << "vLastSetPos[" << i << "] = " << rbt.vLastSetPos[i] << std::endl;
    // }  //检查vLastSetPos数值
    cout<<"rbt.mfJointCmdPos:"<<rbt.mfJointCmdPos<<endl;  
    //是一个函数，通常用于将机器人关节的目标位置设置到电机驱动器中。它可能包括将目标关节角度发送到各个电机，从而驱动机器人关节到达指定的角度。
    
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
            struct timeval startTime,endTime;
            double timeUse;
            gettimeofday(&startTime,NULL);
            //If stay static, annotate below one line.

            rbt.NextStep();
            // cout<<"rbt.mfCompensationZ:"<<rbt.mfCompensationZ<<endl;
            Matrix<float,4,3> mfLegCmdCompPos1=rbt.FnnStepModify();
            //cout<<"mfLegCmdCompPos "<<mfLegCmdCompPos1<<endl;
            rbt.InverseKinematics(mfLegCmdCompPos1);	

            //rbt.AirControl();
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
    }
 
}

void *runImpCtller(void *data) //线程3
{
    struct timeval startTime,endTime;
    double timeUse;
    int run_times=0;    // for debugging 用于调试的变量，可能用来跟踪控制循环的运行次数，虽然在代码中没有使用。

    while(rbt.bInitFlag == 0) //wait for initial 这段代码会等待 rbt.bInitFlag 变为1，表示系统已经完成初始化，之后才会开始执行控制循环。usleep(1e2) 会让线程休眠100微秒，避免占用过多CPU资源。
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
            motors.getVelocity();//通过调用 getTorque(), getPosition() 和 getVelocity() 方法从电机获取当前的扭矩、位置和速度信息，通常这些信息来自传感器或编码器。

            /* update the data IMP need */
            rbt.UpdatejointPresPosAndVel(motors.present_position);//更新关节的当前位置和速度         
            //rbt.UpdatejointPresVel(); //useless,wrong data
            rbt.ForwardKinematics(1);//进行正向运动学计算，推导出末端执行器（如脚或手）的位置。
            rbt.UpdateJacobians();//更新机器人系统中的雅可比矩阵（用于描述关节速度和末端速度之间的关系）。
           // rbt.UpdateFtsPresVel();
            rbt.UpdateFtsPresForce(motors.present_torque);  //更新关节的力矩或力传感器数据。
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
             rbt.InverseKinematics(rbt.mfLegCmdPos); //使用逆运动学算法将 mfLegCmdPos （腿的目标位置）转换为关节角度。这意味着系统根据目标末端位置来计算相应的关节角度
             cout<<endl;
             cout<<"mfLegCmdPos:\n"<<rbt.mfLegCmdPos<<endl;
             cout<<"mfJointCmdPos:\n"<<rbt.mfJointCmdPos<<endl;
            //  cout<<"mfJointCmdPos:\n"<<rbt.vLastSetPos<<endl;
            // cout<<"mfLegCmdPos: \n"<<rbt.mfLegCmdPos<<endl;
            // cout<<"target_pos: \n"<<rbt.mfTargetPos<<endl;
            // cout<<"legPresPos: \n"<<rbt.mfLegPresPos<<"; \nxc: \n"<<rbt.xc<<endl;
            //cout<<"force:"<<endl<<rbt.mfForce.transpose()<<endl;
            // cout<<"xc_dotdot: \n"<<rbt.mfXcDotDot<<"; \nxc_dot: \n"<<rbt.mfXcDot<<"; \nxc: \n"<<rbt.mfXc<<endl;
            // cout<<endl;

            /*      Set joint angle      */
            SetPos(rbt.mfJointCmdPos,motors,rbt.vLastSetPos);//调用 SetPos 函数，将计算出的关节角度命令应用到电机系统中，从而控制机器人的动作

            /*      Impedance control      */
            // for(int i=0; i<4; i++)  
            //     for(int j=0;j<3;j++)
            //         SetTorque[i*3+j] = rbt.target_torque(j,i);
            // motors.setTorque(SetTorque); 

            gettimeofday(&endTime,NULL);
            timeUse = 1e6*(endTime.tv_sec - startTime.tv_sec) + endTime.tv_usec - startTime.tv_usec;
            if(timeUse < 1e4)
                usleep(1.0/loopRateImpCtller*1e6 - (double)(timeUse) - 10); //通过记录控制循环的开始和结束时间来计算每次循环的执行时间 timeUse，并确保循环以恒定的频率运行。这里的 loopRateImpCtller 是目标的控制循环频率，通过 usleep 函数控制线程的休眠时间以保证这个频率
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
    // ret = pthread_create(&th1,NULL,udpConnect,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread1 error!\n");
	// 	exit(1);
	// }
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
    // ret = pthread_create(&th4,NULL,dataSave,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread4 error!\n");
	// 	exit(1);
	// }
    //  ret = pthread_create(&th5,NULL,SvUpdate,NULL);
    // if(ret != 0)
	// {
	// 	printf("create pthread4 error!\n");
	// 	exit(1);
	// }
    
    // pthread_join(th1, NULL);
    pthread_join(th2, NULL);// zuichu
    pthread_join(th3, NULL);
    // pthread_join(th4, NULL);
    // pthread_join(th5, NULL);
    while(1);

    
    
    return 0;
}

