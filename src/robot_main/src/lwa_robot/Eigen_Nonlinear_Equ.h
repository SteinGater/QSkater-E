#ifndef Eigen_Nonlinear_Equ
#define Eigen_Nonlinear_Equ

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include <time.h>
#include <stdint.h>
#include <sys/types.h>
#include <iostream>

#include "eigen3/Eigen/Eigen"
#include "eigen3/unsupported/Eigen/NonLinearOptimization"
#include "eigen3/unsupported/Eigen/NumericalDiff"

#include "CJW_Math.h"

//#define USING_ANALYSIS_DIFFERNET 0

using namespace Eigen;
/*****************************cost function***********************************************/
template <class T>
class CostFunction
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar = typename T::Scalar;
    static constexpr auto InputsAtCompileTime =Eigen::Dynamic;
    static constexpr auto ValuesAtCompileTime =Eigen::Dynamic;
    using InputType = Eigen::Matrix<Scalar, InputsAtCompileTime, 1>;
    using ValueType = Eigen::Matrix<Scalar, ValuesAtCompileTime, 1>;
    using JacobianType = Eigen::Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime>;
private:
    T &mSolver;
public:
    CostFunction()=delete;
    explicit CostFunction(T &solver):mSolver(solver)
    {
    }
    //Eigen最优化类需要的函数,优化个数，代价函数个数
    int inputs() const { return mSolver.inputs(); }
    int values() const { return mSolver.values(); }
    /***代价函数*/
    inline int operator()(const InputType &x, ValueType &y) const
    {
        mSolver.cost_function(x,y);
        return 0;
    }
#ifdef USING_ANALYSIS_DIFFERNET
    /**解析微分*/
    inline  int df(const InputType &x, ValueType &dy) const
    {
        mSolver.df_function(x,dy);
        return 0;
    }
#endif

};

/*****************************OptimizerSolver function***********************************************/
class NonLinerEquSolver
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    using Scalar=double;
    using InputType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    using ValueType = Eigen::Matrix<Scalar, Eigen::Dynamic, 1>;
    Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CostFunction<NonLinerEquSolver>>,double> solver;
private:
    friend CostFunction<NonLinerEquSolver>;
    int StateXN;
    int CostFN;
    CostFunction<NonLinerEquSolver> costf;
    Eigen::NumericalDiff<CostFunction<NonLinerEquSolver>> numDiff;
public:
    NonLinerEquSolver()=delete;
    explicit NonLinerEquSolver(int XN,int FN)
    :StateXN(XN),CostFN(FN),costf(*this),numDiff(costf),solver(numDiff)
    {
        solver.parameters.epsfcn=0.0001;
        solver.parameters.maxfev=10;
        solver.parameters.xtol=0.0001;
        solver.parameters.ftol=0.0001;
        solver.parameters.factor=100;
    }
    //Eigen最优化类需要的函数,优化个数，代价函数个数
    int inputs() const {return StateXN;}
    int values() const {return CostFN;}
    //代价函数-继承重构使用
    virtual void cost_function(const InputType &x,ValueType &y)
    {
        //test
        for(int ii=0;ii<CostFN;ii++)
        {
            y(ii)=sin(x(ii))-0.2;
        }
    }
    //解析微分-继承重构使用
    virtual void df_function(const InputType &x,ValueType &dy)
    {
        //test
        for(int ii=0;ii<CostFN;ii++)
        {
            dy(ii)=cos(x(ii));
        }
    }
    /**求解最优化************************************/
    virtual int Solver(Eigen::Matrix<double, Eigen::Dynamic, 1> &x0)
    {
        //CostFunction<NonLinerEquSolver> costf(*this);
        //Eigen::NumericalDiff<CostFunction<NonLinerEquSolver>> numDiff(costf);
        //Eigen::LevenbergMarquardt<Eigen::NumericalDiff<CostFunction<NonLinerEquSolver>>,double> solver(numDiff);
        //std::cout<<"running"<<std::endl;
        int ret=solver.lmder1(x0);
        //std::cout<<"x=::"<<x0.transpose()<<"iter::"<<solver.iter<<std::endl;
        return ret;
    }
};


#endif
