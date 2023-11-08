#################the robot state
Header header	#定义时间戳

int32 Robot_State
#int32 CONTROL_EMERGENCY       =-1 #紧急停止
#int32 CONTROL_STOP            =0  #正常停止
#int32 CONTROL_STAY            =1  #基本运动
#int32 CONTROL_MOVE            =2  #步态移动
#int32 CONTROL_BOUNDING        =3  #步态跳跃
#int32 CONTROL_CHANGE          =10 #分支足式/轮式模式切换
#int32 CONTROL_TEST            =100 #机身六自由度运动测试

int32 Simulation_Flag #仿真标志位置
#int32 REALWORLD=0
#int32 SIMULATIONWORLD=1

int32 Control_Model #控制模式
#int32 CONTROL_MODE_POSITION=0
#int32 CONTROL_MODE_VELOCITY=1
#int32 CONTROL_MODE_FORCE=2

int32 Gait_Type #步态类型
#int32 GAIT_TROT=0
#int32 GAIT_PACE=1
#int32 GAIT_BOUNDING=2

robot_msgl/RobotStruct ExpRobot #期望机器人状态
robot_msgl/RobotStruct RealRobot #实际机器人状态

int32 design_error
#int32 GAIT_ERROR_ROBOT_STATE=1
#int32 GAIT_ERROR_LEG_STATE=2
#int32 GAIT_ERROR_INV_POSITION=3
#int32 GAIT_ERROR_INV_VELOCITY=4
#int32 GAIT_ERROR_INV_FORCE=5

int32 body_error
#int32 ERROR_BODY_ORI_OVER=1
#int32 ERROR_BODY_X_OVER=2
#int32 ERROR_BODY_Y_OVER=3
#int32 ERROR_BODY_Z_OVERMAX=4
#int32 ERROR_BODY_z_OVERMIN=5

int32 leg_error_num #0x1111
