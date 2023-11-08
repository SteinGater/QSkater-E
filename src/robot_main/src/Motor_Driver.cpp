#include "Motor_Driver.h"

/***************************定义常量************************/
robot_msgl::MotorStruct UserMotorExp[User_MainBranchN][MOTOR_BRANCHN_N];
robot_msgl::MotorStruct UserMotorReal[User_MainBranchN][MOTOR_BRANCHN_N];
char User_MotorIDMap[User_MainBranchN][MOTOR_BRANCHN_N] = {
    {0, 1, 2},
    {0, 1, 2},
    {0, 1, 2},
    {0, 1, 2}};
double User_MotorZERO[User_MainBranchN][MOTOR_BRANCHN_N] = {
    {2.887/9,6.136/9,0.192/9},
    {4.0/9,4.899/9,2.061/9},
    {1.119/9,5.589/9,0.147/9},
    {6.445/9,3.167/9,4.419/9}
};
double User_Motor_Original[User_MainBranchN][MOTOR_BRANCHN_N] = {0};
int User_MotorDIR[User_MainBranchN][MOTOR_BRANCHN_N] = {
    {1, -1, -1},
    {1, 1, 1},
    {-1, 1, 1},
    {-1, -1, -1}};
int User_MotorFunction[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};
/**************************电机PID******************************/
double User_Motor_Pos_KP=0.05;//0.5
double User_Motor_Pos_KW=3;//0.5
double User_Motor_Vel_KP=0.001;//0.5
double User_Motor_Vel_KW=3;//0.5
double User_Motor_Tor_KP=0.0001;//0.5
double User_Motor_Tor_KW=3;//0.5

//摩擦有关的参数
double User_MotorFrictionZero[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0}, //{0, 0, 0.1},
        {0, 0, 0}, //{0, 0, 0.1},
        {0, 0, 0}, //{0, 0, 0.2},
        {0, 0, 0}, //{0, 0, 0.1},
};
double User_MotorFrictionStaticUp[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0}, //{0.2, 0.2, 0.8},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};
double User_MotorFrictionStaticDown[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0}, //{0.1, 0.1, 0.2},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};
double User_MotorFrictionDynUp[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};
double User_MotorFrictionDynDown[User_MainBranchN][MOTOR_BRANCHN_N] =
    {
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
        {0, 0, 0},
};
//硬件和中断事件
int Motor_fdi[User_MainBranchN];
int epfd[4];
struct epoll_event eventTest[4];

/************************************************UserFunction*******************************************************/
/***********************************Motor Controller for Driver***********************************/
int Inital_Motor_Connect(char **seriel)
{
    int Ret;
    Motor_fdi[0] = open_set(seriel[0]);
    if (Motor_fdi[0] <= 0)
    {
        printf("Motor leg 1 Serial Open Fail!\n");
        return ERROR;
    }
    std::cout << "Info: handle of serial0 : " << Motor_fdi[0] << std::endl;
    epfd[0] = epoll_create(1);
    eventTest[0].events = EPOLLIN;
    eventTest[0].data.fd = Motor_fdi[0];
    Ret = epoll_ctl(epfd[0], EPOLL_CTL_ADD, Motor_fdi[0], &eventTest[0]);
    if (Ret != 0)
    {
        printf("[ERROR] Serial communication: epoll set error: %d\n", Ret);
    }

    Motor_fdi[1] = open_set(seriel[1]);
    if (Motor_fdi[1] <= 0)
    {
        printf("Motor leg 2 Serial Open Fail!\n");
        return ERROR;
    }
    std::cout << "Info: handle of serial1 : " << Motor_fdi[1] << std::endl;
    epfd[1] = epoll_create(1);
    eventTest[1].events = EPOLLIN;
    eventTest[1].data.fd = Motor_fdi[1];
    Ret = epoll_ctl(epfd[1], EPOLL_CTL_ADD, Motor_fdi[1], &eventTest[1]);
    if (Ret != 0)
    {
        printf("[ERROR] Serial communication: epoll set error: %d\n", Ret);
    }

    Motor_fdi[2] = open_set(seriel[2]);
    if (Motor_fdi[2] <= 0)
    {
        printf("Motor leg 2 Serial Open Fail!\n");
        return ERROR;
    }
    std::cout << "Info: handle of serial1 : " << Motor_fdi[2] << std::endl;
    epfd[2] = epoll_create(1);
    eventTest[2].events = EPOLLIN;
    eventTest[2].data.fd = Motor_fdi[2];
    Ret = epoll_ctl(epfd[2], EPOLL_CTL_ADD, Motor_fdi[2], &eventTest[2]);
    if (Ret != 0)
    {
        printf("[ERROR] Serial communication: epoll set error: %d\n", Ret);
    }

    Motor_fdi[3] = open_set(seriel[3]);
    if (Motor_fdi[3] <= 0)
    {
        printf("Motor leg 2 Serial Open Fail!\n");
        return ERROR;
    }
    std::cout << "Info: handle of serial1 : " << Motor_fdi[3] << std::endl;
    epfd[3] = epoll_create(1);
    eventTest[3].events = EPOLLIN;
    eventTest[3].data.fd = Motor_fdi[3];
    Ret = epoll_ctl(epfd[3], EPOLL_CTL_ADD, Motor_fdi[3], &eventTest[3]);
    if (Ret != 0)
    {
        printf("[ERROR] Serial communication: epoll set error: %d\n", Ret);
    }
    return RIGHT;
}
int Motor_SendRec_One(int Func, int branch, int body, int fd_com, int epfd) //电机收发一次
{
    MOTOR_send motor_ss;
    motor_ss.id = UserMotorExp[branch][body].ID;            //motor ID
    motor_ss.mode = UserMotorExp[branch][body].Enable * 10; //switch to servo mode
    motor_ss.T = 0;                                         //Nm, T<255.9
    motor_ss.W = 0;                                         //rad/s, W<511.9
    motor_ss.Pos = 0;                                       //rad, Pos<131071.9
    motor_ss.K_W = 0;                                       //K_W<63.9     value should around 3
    motor_ss.K_P = 0;                                       //K_P<31.9     value should around 0.1

    double thisfriction = Motor_Get_Friction(branch, body);
    motor_ss.Pos = (UserMotorExp[branch][body].angle * User_MotorDIR[branch][body] + User_MotorZERO[branch][body]) * MOTOR_REDUCTION_RATIO;
    motor_ss.W = UserMotorExp[branch][body].velocity * MOTOR_REDUCTION_RATIO * User_MotorDIR[branch][body];
    if (Func == MOTORCOMMAND_POSITION)
    {
        motor_ss.K_P = User_Motor_Pos_KP;
        motor_ss.K_W = User_Motor_Pos_KW;
    }
    else if (Func == MOTORCOMMAND_VELOCITY)
    {
        motor_ss.K_P = User_Motor_Vel_KP;
        motor_ss.K_W = User_Motor_Vel_KW;
        motor_ss.T = (UserMotorExp[branch][body].torque - thisfriction) / MOTOR_REDUCTION_RATIO * User_MotorDIR[branch][body];
    }
    else if (Func == MOTORCOMMAND_TORQUE)
    {
        motor_ss.K_P = User_Motor_Tor_KP;
        motor_ss.K_W = User_Motor_Tor_KW;
        motor_ss.T = (UserMotorExp[branch][body].torque - thisfriction) / MOTOR_REDUCTION_RATIO * User_MotorDIR[branch][body];
    }
    else
    {
        motor_ss.T = 0 / MOTOR_REDUCTION_RATIO * User_MotorDIR[branch][body];
    }
    modify_data(&motor_ss);
    MOTOR_recv motor_rr;
    int error = send_recv(fd_com, epfd, &motor_ss, &motor_rr);
    if (error == 11)
    {
        error = RIGHT;
        extract_data(&motor_rr);
        User_Motor_Original[branch][body] = motor_rr.Pos; //记录原始数据初始化使用
        //using data to control
        UserMotorReal[branch][body].ID = motor_rr.motor_id;
        UserMotorReal[branch][body].Enable = 1.0 * motor_rr.mode / 10;
        UserMotorReal[branch][body].angle = (motor_rr.Pos / MOTOR_REDUCTION_RATIO - User_MotorZERO[branch][body]) * User_MotorDIR[branch][body];
        UserMotorReal[branch][body].velocity = motor_rr.LW / MOTOR_REDUCTION_RATIO / User_MotorDIR[branch][body];
        UserMotorReal[branch][body].error = motor_rr.MError;
        //using data to show
        UserMotorReal[branch][body].accelation = (motor_rr.Acc / MOTOR_REDUCTION_RATIO / User_MotorDIR[branch][body]);
        UserMotorReal[branch][body].torque = (motor_rr.T * MOTOR_REDUCTION_RATIO / User_MotorDIR[branch][body]) + thisfriction;
    }
    else
    {
        error = ERROR;
    }
    return error;
}

int Motor_Joint_Initial_Calibration(void)
{
    double angleone = 2 * M_PI / MOTOR_REDUCTION_RATIO;
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
        {
            double exp0 = (User_Motor_Calibration[branchi][bodyi] + User_Joint_ZERO[branchi][bodyi] / User_MotorDIR[branchi][bodyi]);
            double thes = exp0 - (int)(exp0 / angleone) * angleone;
            if (thes < 0)
            {
                thes = thes + angleone;
            }
            if (thes >= angleone / 2)
            {
                if ((thes - User_Motor_Original[branchi][bodyi] / MOTOR_REDUCTION_RATIO) > (angleone / 2))
                {
                    thes = thes - angleone;
                }
            }
            else
            {
                if ((thes - User_Motor_Original[branchi][bodyi] / MOTOR_REDUCTION_RATIO) < (-angleone / 2))
                {
                    thes = thes + angleone;
                }
            }
            User_MotorZERO[branchi][bodyi] = thes - User_Joint_ZERO[branchi][bodyi] / User_MotorDIR[branchi][bodyi];
        }
    }
    return RIGHT;
}
void Motor_Joint_Set_PID(int type,double KP,double KW)
{
    if(type==MOTORCOMMAND_POSITION)
    {
        User_Motor_Pos_KP=KP;User_Motor_Pos_KW=KW;
    }
    else if(type==MOTORCOMMAND_VELOCITY)
    {
        User_Motor_Vel_KP=KP;User_Motor_Vel_KW=KW;
    }
    else if(type==MOTORCOMMAND_TORQUE)
    {
        User_Motor_Tor_KP=KP;User_Motor_Tor_KW=KW;
    }
    else
    {

    }
}
/*******************************************Send and Refresh ALL Motor move***************************************************/
int Motor_SendRec_Func_OneBranch(int branchi, int epfd)
{
    int error = RIGHT;
    for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
    {
        UserMotorExp[branchi][bodyi].ID = User_MotorIDMap[branchi][bodyi];
        error = error + Motor_SendRec_One(User_MotorFunction[branchi][bodyi], branchi, bodyi, Motor_fdi[branchi], epfd);
    }
    if (error)
    {
        printf("ERROR: Leg %d Com is error %d\n", branchi, error);
    }
    return error;
}
void Motor_SendRec_Func_ALL(void)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        Motor_SendRec_Func_OneBranch(branchi, epfd[0]);
    }
}
void Motor_Set_Func_OneBranch(int branchi, int type)
{
    for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
    {
        User_MotorFunction[branchi][bodyi] = type;
    }
}
void Motor_Set_Func_ALL(int type)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        Motor_Set_Func_OneBranch(branchi, type);
    }
}
void Set_Motor_One_Branchi_Enable(int branchi, int model)
{
    for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
    {
        UserMotorExp[branchi][bodyi].Enable = model;
    }
}
void Set_Motor_ALL_Enable(int model)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        Set_Motor_One_Branchi_Enable(branchi, model);
    }
}

/****************************关节摩擦模型*******************************/
double Motor_Get_Friction(int branchi, int bodyi)
{
    double thisfriction = 0;
    double FrictionZ = -fabs(User_MotorFrictionZero[branchi][bodyi]);
    double StaticFront = fabs(User_MotorFrictionStaticUp[branchi][bodyi]);
    double StaticBack = fabs(User_MotorFrictionStaticDown[branchi][bodyi]);
    double DynFront = fabs(User_MotorFrictionDynUp[branchi][bodyi]);
    double DynBack = fabs(User_MotorFrictionDynDown[branchi][bodyi]);
    if (UserMotorReal[branchi][bodyi].angle < 0)
    {
        FrictionZ = -FrictionZ;
        StaticFront = fabs(User_MotorFrictionStaticUp[branchi][bodyi]);
        StaticBack = fabs(User_MotorFrictionStaticDown[branchi][bodyi]);
        DynFront = fabs(User_MotorFrictionDynUp[branchi][bodyi]);
        DynBack = fabs(User_MotorFrictionDynDown[branchi][bodyi]);
    }
    if (UserMotorReal[branchi][bodyi].velocity >= MOTOR_FRICTION_VEL_ZERO)
    {
        thisfriction = -DynFront;
    }
    else if (UserMotorReal[branchi][bodyi].velocity <= -MOTOR_FRICTION_VEL_ZERO)
    {
        thisfriction = DynBack;
    }
    else //静摩擦系数
    {
        if (UserMotorExp[branchi][bodyi].accelation > MOTOR_FRICTION_ACC_ZERO)
        {
            thisfriction = -StaticFront;
        }
        else if (UserMotorExp[branchi][bodyi].accelation < -MOTOR_FRICTION_ACC_ZERO)
        {
            thisfriction = StaticBack;
        }
        else
        {
        }
    }
    return (thisfriction + FrictionZ);
}
