######define Tip state
float64[3] Rw #姿态指数映射
float64[3] P  #位置
float64[6] V  #广义速度 角速度+线速度
float64[6] A  #广义加速度 角加速度+线速度
float64[6] F  #力旋量（反向）力矩+力
