#include "UserRobotStructure.h"

using namespace std;

/***************************CJW_BasedJoint structure****************************************************/
struct CJW_BasedJoint LegA_Joint1 = {
    name:"LegA_J1",Angle:0,Velocity:0,Accelation:0,Force:0,
    AngleLimit:{-1.4,1.4},
    VelocityLimit:{-User_Joint_MaxV,User_Joint_MaxV},
    AccelationLimit:{-User_Joint_MaxA,User_Joint_MaxA},
    ForceLimit:{-User_Joint_MaxF,User_Joint_MaxF}
};
struct CJW_BasedJoint LegA_Joint2 = {
    name:"LegA_J2",Angle:0,Velocity:0,Accelation:0,Force:0,
    AngleLimit:{-1.5,1.5},
    VelocityLimit:{-User_Joint_MaxV,User_Joint_MaxV},
    AccelationLimit:{-User_Joint_MaxA,User_Joint_MaxA},
    ForceLimit:{-User_Joint_MaxF,User_Joint_MaxF}
};
struct CJW_BasedJoint LegA_Joint3 = {
    name:"LegA_J3",Angle:0,Velocity:0,Accelation:0,Force:0,
    AngleLimit:{-2.7,2.7},
    VelocityLimit:{-User_Joint_MaxV,User_Joint_MaxV},
    AccelationLimit:{-User_Joint_MaxA,User_Joint_MaxA},
    ForceLimit:{-User_Joint_MaxF,User_Joint_MaxF}
};
struct CJW_BasedJoint LegA_JointD = {
    name:"LegA_JD",Angle:0,Velocity:0,Accelation:0,Force:0,
    AngleLimit:{-M_PI,M_PI},
    VelocityLimit:{-User_Joint_MaxV,User_Joint_MaxV},
    AccelationLimit:{-User_Joint_MaxA,User_Joint_MaxA},
    ForceLimit:{-User_Joint_MaxF,User_Joint_MaxF}
};
struct CJW_BasedJoint LegA_JointW = {
    name:"LegA_JW",Angle:0,Velocity:0,Accelation:0,Force:0,
    AngleLimit:{-M_PI*100000000,M_PI*100000000},
    VelocityLimit:{-User_Wheel_MaxV,User_Wheel_MaxV},
    AccelationLimit:{-User_Joint_MaxA,User_Joint_MaxA},
    ForceLimit:{-User_Joint_MaxF,User_Joint_MaxF}
};

/***************************CJW_RigidBody structure****************************************************/
struct CJW_RigidBody User_Body0 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    M:{{0.0347,0,0,       0,0,0},
        {0,0.093,0,       0,0,0},
        {0,0,0.1128,      0,0,0},
        {0,0,0,       User_MainBody_Mass,0,0},
        {0,0,0,       0,User_MainBody_Mass,0},
        {0,0,0,       0,0,User_MainBody_Mass}}
};
struct CJW_RigidBody User_TypeR_Body1 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,-0.006},
        {0,1,0,0.012},
        {0,0,1,0},
        {0,0,0,1}},
    M:{{0.000586,0,0,       0,0,0},
       {0,0.000872,0,       0,0,0},
       {0,0,0.000648,       0,0,0},
       {0,0,0,       User_LEG_Mass1,0,0},
       {0,0,0,       0,User_LEG_Mass1,0},
       {0,0,0,       0,0,User_LEG_Mass1}}
};
struct CJW_RigidBody User_TypeR_Body2 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0.001},
        {0,1,0,-0.062},
        {0,0,1,-0.038},
        {0,0,0,1}},
    M:{{0.0135,0,0,          0,0,0},
       {0,0.008,0.004,       0,0,0},
       {0,0.004,0.007,       0,0,0},
       {0,0,0,       User_LEG_Mass2,0,0},
       {0,0,0,       0,User_LEG_Mass2,0},
       {0,0,0,       0,0,User_LEG_Mass2}}
};
struct CJW_RigidBody User_TypeR_Body3 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0.002},
        {0,1,0,-0.084},
        {0,0,1,-0.14},
        {0,0,0,1}},
    M:{{0.009,0,0,       0,0,0},
       {0,0.006,0.003,       0,0,0},
       {0,0.003,0.0025,      0,0,0},
       {0,0,0,       User_LEG_Mass3,0,0},
       {0,0,0,       0,User_LEG_Mass3,0},
       {0,0,0,       0,0,User_LEG_Mass3}}
};
struct CJW_RigidBody User_TypeL_Body1 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,-0.006},
        {0,1,0,-0.012},
        {0,0,1,0},
        {0,0,0,1}},
    M:{{0.000586,0,0,       0,0,0},
       {0,0.000872,0,       0,0,0},
       {0,0,0.000648,       0,0,0},
       {0,0,0,       User_LEG_Mass1,0,0},
       {0,0,0,       0,User_LEG_Mass1,0},
       {0,0,0,       0,0,User_LEG_Mass1}}
};
struct CJW_RigidBody User_TypeL_Body2 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0.001},
        {0,1,0,0.062},
        {0,0,1,-0.038},
        {0,0,0,1}},
    M:{{0.0135,0,0,          0,0,0},
       {0,0.008,0.004,       0,0,0},
       {0,0.004,0.007,       0,0,0},
       {0,0,0,       User_LEG_Mass2,0,0},
       {0,0,0,       0,User_LEG_Mass2,0},
       {0,0,0,       0,0,User_LEG_Mass2}}
};
struct CJW_RigidBody User_TypeL_Body3 = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0.002},
        {0,1,0,0.084},
        {0,0,1,-0.15},
        {0,0,0,1}},
    M:{{0.009,0,0,       0,0,0},
       {0,0.006,0.003,       0,0,0},
       {0,0.003,0.0025,      0,0,0},
       {0,0,0,       User_LEG_Mass3,0,0},
       {0,0,0,       0,User_LEG_Mass3,0},
       {0,0,0,       0,0,User_LEG_Mass3}}
};
struct CJW_RigidBody User_Type_BodyD = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    M:{{0.001,0,0,       0,0,0},
       {0,0.001,0,       0,0,0},
       {0,0,0.001,       0,0,0},
       {0,0,0,       0,0,0},
       {0,0,0,       0,0,0},
       {0,0,0,       0,0,0}}
};
struct CJW_RigidBody User_Type_BodyW = {
    BodyV:{0,0,0,0,0,0},
    BodyA:{0,0,0,0,0,0},
    BodyF:{0,0,0,0,0,0},
    MG:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    M:{{0.001,0,0,       0,0,0},
       {0,0.001,0,       0,0,0},
       {0,0,0.001,       0,0,0},
       {0,0,0,       0,0,0},
       {0,0,0,       0,0,0},
       {0,0,0,       0,0,0}}
};
/***************************CJW_JointStruct structure****************************************************/
struct CJW_JointStruct User_MainBody0 = {
    name:"Main_body",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,User_LEG_L1+User_LEG_L2+User_LEG_L3+User_MainBody_Z},
        {0,0,0,1}},
    Screw0:{0,0,0,0,0,0},
    parent:NULL,
    Body:User_Body0
};
struct CJW_JointStruct User_MBranch_Type = {
    name:"Main_Branch_Type",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    Screw0:{0,0,0,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_JointStruct User_TypeR_JStruct1 = {
    name:"LegR_Struct1",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    Screw0:{1,0,0,0,0,0},
    parent:NULL,
    Body:User_TypeR_Body1
};
struct CJW_JointStruct User_TypeR_JStruct2 = {
    name:"LegR_Struct2",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,-User_LEG_L1},
        {0,0,0,1}},
    Screw0:{0,1,0,0,0,0},
    parent:NULL,
    Body:User_TypeR_Body2
};
struct CJW_JointStruct User_TypeR_JStruct3 = {
    name:"LegR_Struct3",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,-User_LEG_L2},
        {0,0,0,1}},
    Screw0:{0,1,0,0,0,0},
    parent:NULL,
    Body:User_TypeR_Body3
};
struct CJW_JointStruct User_TypeL_JStruct1 = {
    name:"LegL_Struct1",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    Screw0:{1,0,0,0,0,0},
    parent:NULL,
    Body:User_TypeL_Body1
};
struct CJW_JointStruct User_TypeL_JStruct2 = {
    name:"LegL_Struct2",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,-User_LEG_L1},
        {0,0,0,1}},
    Screw0:{0,1,0,0,0,0},
    parent:NULL,
    Body:User_TypeL_Body2
};
struct CJW_JointStruct User_TypeL_JStruct3 = {
    name:"LegL_Struct3",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,-User_LEG_L2},
        {0,0,0,1}},
    Screw0:{0,1,0,0,0,0},
    parent:NULL,
    Body:User_TypeL_Body3
};
struct CJW_JointStruct User_TypeR_JStructTip = {
    name:"Leg_StructTip",
    G0:{{1,0,0,0},
        {0,1,0,-User_FOOT_Y},
        {0,0,1,-User_LEG_L3},
        {0,0,0,1}},
    Screw0:{0,0,0,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_JointStruct User_TypeL_JStructTip = {
    name:"Leg_StructTip",
    G0:{{1,0,0,0},
        {0,1,0,User_FOOT_Y},
        {0,0,1,-User_LEG_L3},
        {0,0,0,1}},
    Screw0:{0,0,0,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_JointStruct User_TypeR_JStructD = {
    name:"LegL_StructD",
    G0:{{1,0,0,0},
        {0,1,0,-User_FOOT_Y},
        {0,0,1,-User_LEG_L3},
        {0,0,0,1}},
    Screw0:{0,0,1,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_JointStruct User_TypeL_JStructD = {
    name:"LegR_StructD",
    G0:{{1,0,0,0},
        {0,1,0,User_FOOT_Y},
        {0,0,1,-User_LEG_L3},
        {0,0,0,1}},
    Screw0:{0,0,1,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_JointStruct User_Type_JStructW = {
    name:"Leg_StructTipW",
    G0:{{1,0,0,0},
        {0,1,0,0},
        {0,0,1,0},
        {0,0,0,1}},
    Screw0:{0,1,0,0,0,0},
    parent:NULL,
    Body:Virtual_Body
};
struct CJW_TipStruct User_MainTip0[User_MainBranchN];

/*********************************设置机器人控制参数*************************************/
double User_BodyFKp0[6] = {800, 800, 300, 90, 90, 120}; //动力学机身位姿控制比例系数
double User_BodyFKd0[6]={20,20,10,20,20,18};     //动力学机身速度控制比例系数
double User_BodyVKp[6] = {0};
double User_Qua_S[6] = {50.0, 50.0, 20.0, 1.0, 1.5, 3.0}; //最优化权重{40, 30, 20, 1, 5, 5}; 
double User_Qua_W[3] = {0.01, 0.01, 0.0001};  //安全性权重
double User_InsideKp[3] = {50, 50, 0};        //支撑腿内部相对位置控制比例系数
double User_InsideKd[3] = {1, 1, 0};          //支撑腿内部相对速度控制比例系数  
double User_SwingKp[3] = {300, 300, 500};     //{400,400,400}//摆动腿位置控制比例系数
double User_SwingKd[3] = {0, 0, 0};           //{40,40,40};//摆动腿速度控制比例系数

double User_BodyAccMax[6] = {10, 10, 10, 5, 5, 10};  //机身最大加速度
double User_BodyAccMin[6] = {-10, -10, -10, -5, -5, -WORLD_GRAVITY}; //机身最小加速度

/***********************位姿估计*****************************************/
double Robot_PV_Esti_Para[8] = {imu_process_noise_position,imu_process_noise_velocity,
                                foot_process_noise_position,
                                foot_sensor_noise_position,foot_sensor_noise_velocity,
                                foot_height_sensor_noise,
                                extern_sensor_noise_position,extern_sensor_noise_velocity};
                                
/********************************************************
the robot of user
********************************************************/
CJW_Math<double> User_Math;
CJW_LWARobot User_RobotExp;
CJW_LWARobot User_RobotReal;
CJW_Gait_Qua User_Gait_Qua;
CJW_GaitToRobotBased User_Gait;


/********************************************************
the robot  initial the struct setting
********************************************************/
void MyRobot_Init(void)
{
    /***********************************计算关节坐标系下广义惯性矩*******************************/
    // double theMoment[36];
    // struct CJW_JointStruct* therigidbody[7]={&User_MainBody0,
    //             &User_TypeR_JStruct1,&User_TypeR_JStruct2,&User_TypeR_JStruct3,
    //             &User_TypeL_JStruct1,&User_TypeL_JStruct2,&User_TypeL_JStruct3};
    // for(int then=0;then<7;then++)//
    // {
    //     User_Math.MyInertiaBasedGroup((double*)(therigidbody[then]->Body.M),(double*)(therigidbody[then]->Body.MG),theMoment);
    //     User_Math.MyMatrixCopy(36,theMoment,(double*)(therigidbody[then]->Body.M));
    //     //User_Math.Show(6,6,(double*)(therigidbody[then]->Body.M));
    // }
    /******************************************************************************/
    CJW_RobotStructure User_Robot_Struct;
    /*set the main body************************************************************/
    User_Robot_Struct.name="Qua_Robot";
    User_Robot_Struct.BranchN=User_MainBranchN;
    User_Robot_Struct.MainBody=User_MainBody0;
    for(int legi=0;legi<User_Robot_Struct.BranchN;legi++)
    {
        User_Robot_Struct.BranchP[legi]=User_MBranch_Type;
        double theangle=2.0*M_PI*legi/User_Robot_Struct.BranchN-M_PI/4;
        User_Robot_Struct.BranchP[legi].G0[0][3]=User_MainBody_X*sqrt(2)*cos(theangle);
        User_Robot_Struct.BranchP[legi].G0[1][3]=User_MainBody_Y*sqrt(2)*sin(theangle);
        User_Robot_Struct.BranchP[legi].G0[2][3]=-User_MainBody_Z;
    }
    /*set the branch stype************************************************************/
    struct CJW_BranchStructure UserBranchA;
    struct CJW_LWABranch BranchCache;
    //set the based struct
    UserBranchA.name="TypeMamR";
    UserBranchA.BodyN=User_MainBranchBodyN;
    UserBranchA.Joint[0]=LegA_Joint1;
    UserBranchA.Joint[1]=LegA_Joint2;
    UserBranchA.Joint[2]=LegA_Joint3;
    UserBranchA.Joint[3]=LegA_JointD;
    UserBranchA.Joint[4]=LegA_JointW;
    UserBranchA.JStruct[0]=User_TypeR_JStruct1;UserBranchA.JStruct[0].parent=NULL;
    UserBranchA.JStruct[1]=User_TypeR_JStruct2;UserBranchA.JStruct[1].parent=&(UserBranchA.JStruct[0]);
    UserBranchA.JStruct[2]=User_TypeR_JStruct3;UserBranchA.JStruct[2].parent=&(UserBranchA.JStruct[1]);
    UserBranchA.JStruct[3]=User_TypeR_JStructD; UserBranchA.JStruct[3].parent=&(UserBranchA.JStruct[2]);
    UserBranchA.JStruct[4]=User_Type_JStructW; UserBranchA.JStruct[4].parent=&(UserBranchA.JStruct[3]);
    UserBranchA.Model_EN[BRANCH_LEG_MODEL]=1;UserBranchA.Model_EN[BRANCH_WHEEL_MODEL]=1;UserBranchA.Model_EN[BRANCH_ARM_MODEL]=0;
    //set the leg struct
    UserBranchA.BModel[BRANCH_LEG_MODEL].modelstate=BRANCH_LEG_MODEL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JointN=3;
    UserBranchA.BModel[BRANCH_LEG_MODEL].State=BRANCH_SUP_STATE;
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[0]=&(UserBranchA.Joint[0]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[1]=&(UserBranchA.Joint[1]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[2]=&(UserBranchA.Joint[2]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[0]=UserBranchA.JStruct[0];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[0].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[1]=UserBranchA.JStruct[1];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[1].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[2]=UserBranchA.JStruct[2];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[2].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[3]=User_TypeR_JStructTip; UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[3].parent=NULL;
    //set the wheel struct
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].modelstate=BRANCH_WHEEL_MODEL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JointN=3;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].State=BRANCH_SUP_STATE;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[0]=&(UserBranchA.Joint[0]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[1]=&(UserBranchA.Joint[1]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[2]=&(UserBranchA.Joint[2]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[0]=UserBranchA.JStruct[0];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[0].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[1]=UserBranchA.JStruct[1];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[1].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[2]=UserBranchA.JStruct[2];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[2].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[3]=User_TypeR_JStructTip;UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[3].parent=NULL;
    //set the arm struct
    //UserBranchA.BModel[BRANCH_ARM_MODEL].modelstate=BRANCH_ARM_MODEL;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].JointN=3;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].State=ARM_MOVE_A;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].Joint[0]=&(UserBranchA.Joint[0]);
    //UserBranchA.BModel[BRANCH_ARM_MODEL].Joint[1]=&(UserBranchA.Joint[1]);
    //UserBranchA.BModel[BRANCH_ARM_MODEL].Joint[2]=&(UserBranchA.Joint[2]);
    //UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[0]=UserBranchA.JStruct[0];UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[0].parent=NULL;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[1]=UserBranchA.JStruct[1];UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[1].parent=NULL;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[2]=UserBranchA.JStruct[2];UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[2].parent=NULL;
    //UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[3]=User_TypeA_JStructTip; UserBranchA.BModel[BRANCH_ARM_MODEL].JStruct[3].parent=NULL;
    
    BranchCache.SetStruct(&UserBranchA);
    BranchCache.GetStruct(&(User_Robot_Struct.BranchBody[0]));
    BranchCache.GetStruct(&(User_Robot_Struct.BranchBody[3]));

    //set the based struct
    UserBranchA.name="TypeMamR";
    UserBranchA.BodyN=User_MainBranchBodyN;
    UserBranchA.Joint[0]=LegA_Joint1;
    UserBranchA.Joint[1]=LegA_Joint2;
    UserBranchA.Joint[2]=LegA_Joint3;
    UserBranchA.Joint[3]=LegA_JointD;
    UserBranchA.Joint[4]=LegA_JointW;
    UserBranchA.JStruct[0]=User_TypeL_JStruct1;UserBranchA.JStruct[0].parent=NULL;
    UserBranchA.JStruct[1]=User_TypeL_JStruct2;UserBranchA.JStruct[1].parent=&(UserBranchA.JStruct[0]);
    UserBranchA.JStruct[2]=User_TypeL_JStruct3;UserBranchA.JStruct[2].parent=&(UserBranchA.JStruct[1]);
    UserBranchA.JStruct[3]=User_TypeL_JStructD; UserBranchA.JStruct[3].parent=&(UserBranchA.JStruct[2]);
    UserBranchA.JStruct[4]=User_Type_JStructW; UserBranchA.JStruct[4].parent=&(UserBranchA.JStruct[3]);
    UserBranchA.Model_EN[BRANCH_LEG_MODEL]=1;UserBranchA.Model_EN[BRANCH_WHEEL_MODEL]=1;UserBranchA.Model_EN[BRANCH_ARM_MODEL]=0;
    //set the leg struct
    UserBranchA.BModel[BRANCH_LEG_MODEL].modelstate=BRANCH_LEG_MODEL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JointN=3;
    UserBranchA.BModel[BRANCH_LEG_MODEL].State=BRANCH_SUP_STATE;
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[0]=&(UserBranchA.Joint[0]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[1]=&(UserBranchA.Joint[1]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].Joint[2]=&(UserBranchA.Joint[2]);
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[0]=UserBranchA.JStruct[0];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[0].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[1]=UserBranchA.JStruct[1];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[1].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[2]=UserBranchA.JStruct[2];UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[2].parent=NULL;
    UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[3]=User_TypeL_JStructTip; UserBranchA.BModel[BRANCH_LEG_MODEL].JStruct[3].parent=NULL;
    //set the wheel struct
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].modelstate=BRANCH_WHEEL_MODEL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JointN=3;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].State=BRANCH_SUP_STATE;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[0]=&(UserBranchA.Joint[0]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[1]=&(UserBranchA.Joint[1]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].Joint[2]=&(UserBranchA.Joint[2]);
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[0]=UserBranchA.JStruct[0];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[0].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[1]=UserBranchA.JStruct[1];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[1].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[2]=UserBranchA.JStruct[2];UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[2].parent=NULL;
    UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[3]=User_TypeL_JStructTip;UserBranchA.BModel[BRANCH_WHEEL_MODEL].JStruct[3].parent=NULL;

    BranchCache.SetStruct(&UserBranchA);
    BranchCache.GetStruct(&(User_Robot_Struct.BranchBody[1]));
    BranchCache.GetStruct(&(User_Robot_Struct.BranchBody[2]));
    /*******Inial robot**********************************/
    User_RobotExp.SetStruct(&User_Robot_Struct);
    //User_RobotExp.ShowStruct();
    User_RobotReal.SetStruct(&User_Robot_Struct);
    User_RobotExp.CJW_Init();
    User_RobotReal.CJW_Init();
    User_RobotExp.RobotAngleToAllStateBody();
    User_RobotExp.MainAnglePTransToTipG();
    User_RobotExp.MainAngleVTransToTipV();
    User_RobotExp.GetMainModelTip(User_MainTip0);
    User_RobotReal.RobotAngleToAllStateBody();
    User_RobotReal.MainAnglePTransToTipG();
    User_RobotReal.MainAngleVTransToTipV();

    /**********Inital gait*******************************/
    User_Gait.Init((CJW_LWARobot*)(&User_RobotExp),(CJW_LWARobot*)(&User_RobotReal),(CJW_GaitBased*)(&User_Gait_Qua));
    User_Gait.SetControlParameter(User_BodyFKp0,User_BodyFKd0,User_BodyVKp,User_Qua_S,User_Qua_W,User_BodyAccMax,User_BodyAccMin,
                                    User_InsideKp,User_InsideKd,User_SwingKp,User_SwingKd); 
    User_Gait.SetInitState();
    

    /*************test*************************/
    /*double body_temp[3] = {0};
    User_RobotExp.GetMainBodyP(body_temp);
    body_temp[2]=0.3;
    int statestate[4]={BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL,BRANCH_WHEEL_MODEL};
    //User_RobotExp.SetBranchModel(statestate);
    User_RobotExp.SetMainBodyP(body_temp);
    User_RobotExp.MainTipGTransToAngleP();
    double expdir[4]={0.7,-0.3,0,-0.5};
    User_RobotExp.SetAllWheelDirection(expdir);
    double realdir[4]={0};
    User_RobotExp.GetAllWheelDirection(realdir);
    printf("test:\n");
    User_Math.Show(1,4,expdir);
    User_Math.Show(1,4,realdir);
    double allangle[100]={0};
    User_RobotExp.GetALLAngleP(allangle);
    printf("joint angle:\n");
    User_Math.Show(4,BRANCH_BODY_MAX,allangle);
    printf("test angle: %3f\n",atan2(0,1));*/

};
/***********************监测机身位置姿态是否正常******************************/
int  MyRobot_Check_Body(CJW_LWARobot* robot)
{
    double theR[3][3]={1,0,0, 0,1,0, 0,0,1};
    double theP[3]={0};
    int error=RIGHT;
    robot->GetMainBodyR(theR);
    robot->GetMainBodyP(theP);
    double thew[3]={0};User_Math.MyRToExponent3((double*)theR,thew);
    if((thew[0]*thew[0]+thew[1]*thew[1])>(User_Body_Limit_W0*User_Body_Limit_W0))
    {
        error=ERROR_BODY_ORI_OVER;
        printf("the body orientation over limit\n");
    }
    else if(fabs(theP[0])>User_Body_Limit_XY)
    {
        error=ERROR_BODY_X_OVER;
        printf("the body X over limit\n");
    }
    else if(fabs(theP[1])>User_Body_Limit_XY)
    {
        error=ERROR_BODY_Y_OVER;
        printf("the body Y over limit\n");
    }
    else if(theP[2]>User_Body_Limit_ZMAX)
    {
        error=ERROR_BODY_Z_OVERMAX;
        printf("the body Z over max limit\n");
    }
    else if(theP[2]<=0)
    {
        error=ERROR_BODY_Z_OVERMIN;
        printf("the body Z over min limit\n");
    }
    else
    {
        error=RIGHT;
    }
    return error;
}

/*******************************public robot state*************************************************/
void MapRobotToPublicState(CJW_LWARobot *in, robot_msgl::RobotStruct *out)
{
    //get robot model
    out->Robot_Model = in->GetRobotModel();
    //get robot body
    struct CJW_JointStruct tempbody;
    in->GetMainBodyState(&tempbody);
    double screw0[6];
    User_Math.MyGToExponent4((double *)(tempbody.G0), screw0);
    for (int i = 0; i < 3; i++)
    {
        out->body.Rw[i] = screw0[i];
        out->body.P[i] = tempbody.G0[i][3];
    }
    for (int i = 0; i < 6; i++)
    {
        out->body.V[i] = tempbody.Body.BodyV[i];
        out->body.A[i] = tempbody.Body.BodyA[i];
        out->body.F[i] = tempbody.Body.BodyF[i];
    }
    //get branch tip state
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        out->BranchModel[branchi] = in->GetBranchiModel(branchi);
        out->BranchState[branchi] = in->GetBranchiState(branchi);
        struct CJW_TipStruct TipState0;
        in->GetMainModelTipOne(branchi, &TipState0);
        double TipR0[9] = {TipState0.G[0][0], TipState0.G[0][1], TipState0.G[0][2],
                           TipState0.G[1][0], TipState0.G[1][1], TipState0.G[1][2],
                           TipState0.G[2][0], TipState0.G[2][1], TipState0.G[2][2]};
        double screw1[3];
        User_Math.MyRToExponent3(TipR0, screw1);
        for (int i = 0; i < 3; i++)
        {
            out->BranchTip[branchi].Rw[i] = screw1[i];
            out->BranchTip[branchi].P[i] = TipState0.G[i][3];
        }
        struct CJW_TipStruct TipState1 = TipState0;
        User_Math.MyRCompositionw(TipR0, TipState0.V, TipState1.V);
        User_Math.MyRCompositionw(TipR0, &(TipState0.V[3]), &(TipState1.V[3]));
        User_Math.MyRCompositionw(TipR0, TipState0.A, TipState1.A);
        User_Math.MyRCompositionw(TipR0, &(TipState0.A[3]), &(TipState1.A[3]));
        User_Math.MyRCompositionw(TipR0, TipState0.F, TipState1.F);
        User_Math.MyRCompositionw(TipR0, &(TipState0.F[3]), &(TipState1.F[3]));
        for (int i = 0; i < 6; i++)
        {
            out->BranchTip[branchi].V[i] = TipState1.V[i];
            out->BranchTip[branchi].A[i] = TipState1.A[i];
            out->BranchTip[branchi].F[i] = TipState1.F[i];
        }
    }
    //get robot motor state
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        struct CJW_BasedJoint temp[User_MainBranchBodyN];
        in->GetBranchiAngleState(branchi, temp);
        for (int bodyi = 0; bodyi < User_MainBranchBodyN; bodyi++)
        {
            int thetemp = branchi * User_MainBranchBodyN + bodyi;
            out->MotorState[thetemp].ID = 10*branchi+bodyi;
            out->MotorState[thetemp].angle = (temp[bodyi].Angle);
            out->MotorState[thetemp].velocity = (temp[bodyi].Velocity);
            out->MotorState[thetemp].accelation = (temp[bodyi].Accelation);
            out->MotorState[thetemp].torque = (temp[bodyi].Force);
        }
    }
}
/*******************************msg to CJW struct*************************************************/
void MapTipMsgToJointStruct(robot_msgl::TipStruct *in, struct CJW_JointStruct *out)
{
    double screw0[6] = {in->Rw[0], in->Rw[1], in->Rw[2], in->P[0], in->P[1], in->P[2]};
    User_Math.MyExponent4ToG(screw0, (double *)(out->G0));
    for (int i = 0; i < 3; i++)
    {
        out->G0[i][3] = in->P[i];
    }
    for (int i = 0; i < 6; i++)
    {
        out->Body.BodyV[i] = in->V[i];
        out->Body.BodyA[i] = in->A[i];
        out->Body.BodyF[i] = in->F[i];
    }
}
void MapTipMsgToTipStruct(robot_msgl::TipStruct *in, struct CJW_TipStruct *out)
{
    double screw0[6] = {in->Rw[0], in->Rw[1], in->Rw[2], in->P[0], in->P[1], in->P[2]};
    User_Math.MyExponent4ToG(screw0, (double *)(out->G));
    for (int i = 0; i < 3; i++)
    {
        out->G[i][3] = in->P[i];
    }
    for (int i = 0; i < 6; i++)
    {
        out->V[i] = in->V[i];
        out->A[i] = in->A[i];
        out->F[i] = in->F[i];
    }
}