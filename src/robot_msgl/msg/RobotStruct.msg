#################the robot state
int32 Robot_Model #模式
#int32 ROBOT_MODEL_MIX=-1
#int32 ROBOT_MODEL_LEG=0
#int32 ROBOT_MODEL_WHEEL=1


robot_msgl/TipStruct body #机身状态

robot_msgl/TipStruct[4] BranchTip #分支末端状态

int32[6] BranchModel    #分支模式：足/轮/臂
#int32 BRANCH_LEG_MODEL=0
#int32 BRANCH_WHEEL_MODEL=1
#int32 BRANCH_ARM_MODEL=2

float64[4] BranchState #分支状态：摆动/支撑
#int32 BRANCH_SWI_STATE=0
#int32 BRANCH_SUP_STATE=1

robot_msgl/MotorStruct[20] MotorState #关节状态

