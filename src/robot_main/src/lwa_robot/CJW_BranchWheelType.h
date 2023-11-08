#ifndef CJW_BRANCHWHEELTYPE
#define CJW_BRANCHWHEELTYPE

#include "CJW_BranchBased.h"
#include "Eigen_Nonlinear_Equ.h"


///***************************定义常量************************/
//define the numerical solver for the leg-wheel inverted kinematics
#define INVERTED_KINE_MERHOD       0 //0:simple, 1:numericial

//define branch state
#define BRANCH_WHEEL_MODEL         1


/***********************************************define model function**********************************************/

/********************************************************
the robot branch is the wheel model
*********************************************************/
class CJW_BranchWheelType: public CJW_BranchBased, public NonLinerEquSolver
{
public:
    using CJW_Vector3=Eigen::Matrix<double,3,1>;
    /*************Constructor****************************************************************/
    CJW_BranchWheelType(struct CJW_BranchModel* Input)
                        :CJW_BranchBased(Input),NonLinerEquSolver(4,4)
    {

    }
    ~CJW_BranchWheelType(void){}
    virtual void CJW_Init(void);
    /************special on function**********************************************************/
    virtual int  AngleFTransToTipF(void);
    //because the inversense only use the position after get angle will compute the orientation of tip
    virtual int  TipGTransToAngleP(double BodyG[4][4]);
    //because the inversense only use the linear velocity after get angle will compute the rorate velocity of tip
    virtual int  TipVTransToAngleV(void);

    /***********optimization function**********************************************************/
    virtual void cost_function(const InputType &x,ValueType &y);
    
protected:
    int JN;
    double LengthL[3];
    double LY;
    /***inverted kinematics**********************************************************/
    int Inverted_Simple(double BodyG[4][4]);
    int Inverted_Numerical(double BodyG[4][4]);
    const double Equ_rw=0.027;
    const double Equ_bw=0.005;
    Eigen::Matrix3d Equ_bodyR;
    CJW_Vector3 Equ_footPN;
    double Equ_footyaw;

private:

};


#endif
