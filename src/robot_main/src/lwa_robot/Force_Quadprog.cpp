#include <stdio.h>
#include <stdlib.h>

#include <math.h>
#include "Force_Quadprog.h"
#include <qpOASES.hpp>

using namespace std;
/***************************定义常量************************/



/*************************************************函数申明****************************************/
int QuadprogTest(void);
int Quadprog_Solve(int Size_N,int Size_M,double *in_H,double *in_g,double *in_lb, double *in_ub,
                   double *in_A,double *in_lbA,double *in_ubA,double *out_x);
int Quadprog_SolveM(int Size_N,int Size_M,double** in_H,double *in_g,double *in_lb, double *in_ub,
                   double **in_A,double *in_lbA,double *in_ubA,double *out_x);

/************************************************定义功能函数***************************************/
/********************************Based Leg*********************************************/
int Quadprog_Solve(int Size_N,int Size_M,double *in_H,double *in_g,double *in_lb, double *in_ub,
                   double *in_A,double *in_lbA,double *in_ubA,double *out_x)
{
    USING_NAMESPACE_QPOASES

    //Setup data of first QP. //
    real_t H[Size_N*Size_N];
    real_t g[Size_N];
    real_t lb[Size_N];
    real_t ub[Size_N];
    real_t A[Size_M*Size_N];
    real_t lbA[Size_M];
    real_t ubA[Size_M];
    /*set the main parament*/
    for(int my_n1=0;my_n1<Size_N;my_n1++)
    {
        for(int my_n2=0;my_n2<Size_N;my_n2++)
        {
            H[my_n1*Size_N+my_n2]=in_H[my_n1*Size_N+my_n2];
        }
        g[my_n1]=in_g[my_n1];
        lb[my_n1]=in_lb[my_n1];
        ub[my_n1]=in_ub[my_n1];
    }
    for(int my_m=0;my_m<Size_M;my_m++)
    {
        for(int my_n=0;my_n<Size_N;my_n++)
        {
            A[my_m*Size_N+my_n]=in_A[my_m*Size_N+my_n];
        }
        lbA[my_m]=in_lbA[my_m];
        ubA[my_m]=in_ubA[my_m];
    }
    /* Setting up QProblem object. */
    QProblem example(Size_N,Size_M);

    Options options;
    options.printLevel=PL_LOW;
    //options.terminationTolerance=0.15;
    example.setOptions( options );

    /* Solve first QP. */
    int_t nWSR = 50;
    if(example.init( H,g,A,lb,ub,lbA,ubA, nWSR)==SUCCESSFUL_RETURN)
    {
        /* Get and print solution of first QP. */
        real_t xOpt[Size_N];
        //real_t yOpt[2+1];
        example.getPrimalSolution(xOpt);
        //example.getDualSolution( yOpt );
        for(int my_n=0;my_n<Size_N;my_n++)
        {
            out_x[my_n]=xOpt[my_n];
        }
        return 0;
    }
    else
    {
        return 1;
    }

}
/********************************Based Quadprog*********************************************/
int Quadprog_SolveM(int Size_N,int Size_M,double** in_H,double *in_g,double *in_lb, double *in_ub,
                   double **in_A,double *in_lbA,double *in_ubA,double *out_x)
{
    USING_NAMESPACE_QPOASES

    //Setup data of first QP. //
    real_t H[Size_N*Size_N];
    real_t g[Size_N];
    real_t lb[Size_N];
    real_t ub[Size_N];
    real_t A[Size_M*Size_N];
    real_t lbA[Size_M];
    real_t ubA[Size_M];
    /*set the main parament*/
    for(int my_n1=0;my_n1<Size_N;my_n1++)
    {
        for(int my_n2=0;my_n2<Size_N;my_n2++)
        {
            H[my_n1*Size_N+my_n2]=in_H[my_n1][my_n2];
        }
        g[my_n1]=in_g[my_n1];
        lb[my_n1]=in_lb[my_n1];
        ub[my_n1]=in_ub[my_n1];
    }
    for(int my_m=0;my_m<Size_M;my_m++)
    {
        for(int my_n=0;my_n<Size_N;my_n++)
        {
            A[my_m*Size_N+my_n]=in_A[my_m][my_n];
        }
        lbA[my_m]=in_lbA[my_m];
        ubA[my_m]=in_ubA[my_m];
    }
    /* Setting up QProblem object. */
    QProblem example(Size_N,Size_M);

    Options options;
    options.printLevel=PL_LOW;
    //options.terminationTolerance=0.15;
    example.setOptions( options );

    /* Solve first QP. */
    int_t nWSR = 50;
    if(example.init( H,g,A,lb,ub,lbA,ubA, nWSR)==SUCCESSFUL_RETURN)
    {
        /* Get and print solution of first QP. */
        real_t xOpt[Size_N];
        //real_t yOpt[2+1];
        example.getPrimalSolution(xOpt);
        //example.getDualSolution( yOpt );
        for(int my_n=0;my_n<Size_N;my_n++)
        {
            out_x[my_n]=xOpt[my_n];
        }
        return 0;
    }
    else
    {
        printf("the Quaprog is error!\n");
        return 1;
    }

}
/****************Test***********************************************************/
int QuadprogTest(void)
{
    USING_NAMESPACE_QPOASES

    //Setup data of first QP. */
    real_t H[2*2] = { 1.0, 0.0, 0.0, 0.5 };
    real_t A[1*2] = { 1.0, 1.0 };
    real_t g[2] = { 1.5, 1.0 };
    real_t lb[2] = { 0.5, -2.0 };
    real_t ub[2] = { 5.0, 2.0 };
    real_t lbA[1] = { -1.0 };
    real_t ubA[1] = { 2.0 };

    /* Setup data of second QP. */
    real_t g_new[2] = { 1.0, 1.5 };
    real_t lb_new[2] = { 0.0, -1.0 };
    real_t ub_new[2] = { 5.0, -0.5 };
    real_t lbA_new[1] = { -2.0 };
    real_t ubA_new[1] = { 1.0 };


    /* Setting up QProblem object. */
    QProblem example( 2,1 );

    Options options;
    options.printLevel=PL_LOW;
    example.setOptions( options );

    /* Solve first QP. */
    int_t nWSR = 10;
    example.init( H,g,A,lb,ub,lbA,ubA, nWSR );

    /* Get and print solution of first QP. */
    real_t xOpt[2];
    real_t yOpt[2+1];
    example.getPrimalSolution( xOpt );
    example.getDualSolution( yOpt );
    //printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
     //       xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

    /* Solve second QP. */
    //nWSR = 10;
    //example.hotstart( g_new,lb_new,ub_new,lbA_new,ubA_new, nWSR );

    /* Get and print solution of second QP. */
    //example.getPrimalSolution( xOpt );
    //example.getDualSolution( yOpt );
    //printf( "\nxOpt = [ %e, %e ];  yOpt = [ %e, %e, %e ];  objVal = %e\n\n",
    //        xOpt[0],xOpt[1],yOpt[0],yOpt[1],yOpt[2],example.getObjVal() );

    //example.printOptions();
    /*example.printProperties();*/

    /*getGlobalMessageHandler()->listAllMessages();*/

    return 0;
}
