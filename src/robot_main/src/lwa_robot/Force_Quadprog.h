#ifndef FORCE_QUADPROG
#define FORCE_QUADPROG

/***************************定义常量************************/



/*************************************************函数申明****************************************/
int QuadprogTest(void);
int Quadprog_Solve(int Size_N,int Size_M,double *in_H,double *in_g,double *in_lb, double *in_ub,
                   double *in_A,double *in_lbA,double *in_ubA,double *out_x);
int Quadprog_SolveM(int Size_N,int Size_M,double** in_H,double *in_g,double *in_lb, double *in_ub,
                   double **in_A,double *in_lbA,double *in_ubA,double *out_x);


#endif
