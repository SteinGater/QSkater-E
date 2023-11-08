#include "Joint_Manage.h"

/*******************************parameter*********************************************/


/*****************************function***********************************************/
/******************************set joint enable********************************/
void Set_Joint_One_Branchi_Enable(int branchi, int model)
{
    Set_Motor_One_Branchi_Enable(branchi,model);
    UserServoExp[branchi].Enable=model;
    UserWheelExp[branchi].Enable=model;
}
void Set_Joint_ALL_Enable(int model)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        Set_Joint_One_Branchi_Enable(branchi, model);
    }
}

/********************************transform the joint to the motor to control robot**********************************/
void MapBranchiJointToMotorJoint(CJW_LWARobot *in, int controlmodel)
{
    int swingmodel = controlmodel;
    int supportingmodel = controlmodel;
    if (controlmodel == MOTORCOMMAND_TORQUE)
    {
        swingmodel = MOTORCOMMAND_VELOCITY;
    }
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        struct CJW_BasedJoint temp[User_MainBranchBodyN];
        in->GetBranchiAngleState(branchi, temp);
        /******map motor**********************************************/
        int themodel = swingmodel;
        if (in->GetBranchiState(branchi) >= BRANCH_BOUNDING_STATE)
        {
            themodel = supportingmodel;
        }
        for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
        {
            UserMotorExp[branchi][bodyi].ID = User_MotorIDMap[branchi][bodyi];
            UserMotorExp[branchi][bodyi].angle = (temp[bodyi].Angle);
            UserMotorExp[branchi][bodyi].velocity = (temp[bodyi].Velocity);
            UserMotorExp[branchi][bodyi].accelation = (temp[bodyi].Accelation);
            UserMotorExp[branchi][bodyi].torque = (temp[bodyi].Force);
            User_MotorFunction[branchi][bodyi] = themodel;
        }
        //奇异位置的特殊化处理
        if (fabs(UserMotorExp[branchi][2].angle) < MOTOR_EXCHANGE_ANGLE)
        {
            //User_MotorFunction[branchi][2]=MOTORCOMMAND_POSITION;
        }
        /******map servo**********************************************/
        UserServoExp[branchi].ID = User_ServoIDMap[branchi];
        UserServoExp[branchi].angle = (temp[3].Angle);
        UserServoExp[branchi].velocity = (temp[3].Velocity);
        UserServoExp[branchi].accelation = (temp[3].Accelation);
        UserServoExp[branchi].torque = (temp[3].Force);
        /******map wheel**********************************************/
        UserWheelExp[branchi].ID = User_WheelIDMap[branchi];
        UserWheelExp[branchi].angle = (temp[4].Angle);
        UserWheelExp[branchi].velocity = (temp[4].Velocity);
        UserWheelExp[branchi].accelation = (temp[4].Accelation);
        UserWheelExp[branchi].torque = (temp[4].Force);        
    }
}
/********************************transform the real joint to robot**********************************/
void MapMotorJointToBranchiJoint(CJW_LWARobot *in)
{
    for (int branchi = 0; branchi < User_MainBranchN; branchi++)
    {
        struct CJW_BasedJoint temp[User_MainBranchBodyN];
        in->GetBranchiAngleState(branchi, temp);
        /******map motor**********************************************/
        for (int bodyi = 0; bodyi < MOTOR_BRANCHN_N; bodyi++)
        {
            UserMotorReal[branchi][bodyi].ID = User_MotorIDMap[branchi][bodyi];
            temp[bodyi].Angle = UserMotorReal[branchi][bodyi].angle;
            temp[bodyi].Velocity = UserMotorReal[branchi][bodyi].velocity;
            temp[bodyi].Accelation = UserMotorReal[branchi][bodyi].accelation;
            temp[bodyi].Force = UserMotorReal[branchi][bodyi].torque;
        }
        /******map servo**********************************************/
        UserServoReal[branchi].ID = User_ServoIDMap[branchi];
        temp[3].Angle = UserServoReal[branchi].angle;
        temp[3].Velocity = UserServoReal[branchi].velocity;
        temp[3].Accelation = UserServoReal[branchi].accelation;
        temp[3].Force = UserServoReal[branchi].torque;
        /******map wheel**********************************************/
        UserWheelReal[branchi].ID = User_WheelIDMap[branchi];
        temp[4].Angle = 0*UserWheelReal[branchi].angle;
        temp[4].Velocity = UserWheelReal[branchi].velocity;
        temp[4].Accelation = UserWheelReal[branchi].accelation;
        temp[4].Force = UserWheelReal[branchi].torque;

        in->SetBranchiAngleState(branchi, temp);
    }
}
