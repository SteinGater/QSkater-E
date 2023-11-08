
#include "CJW_RigidBody.h"

/********************************************************
one rigid robot description in its virtual body
  ******************************************************/
struct CJW_RigidBody Virtual_Body=
{
{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},
{{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}},
{{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0},{0,0,0,0,0,0}}
};


/****************show the structure*********************************************************************/
void Print_CJW_RigidBody(struct CJW_RigidBody* input);
/********************************************************/
void Print_CJW_TipStruct(struct CJW_TipStruct* input);
/********************************************************/
void Print_CJW_BasedJoint(struct CJW_BasedJoint* input);
/********************************************************/
void Print_CJW_JointOffset(struct CJW_JointOffset* input);
/********************************************************/
void Print_CJW_JointStruct(struct CJW_JointStruct* input);
/********************************************************/
void Print_CJW_BranchModel(struct CJW_BranchModel* input);
/********************************************************/
void Print_CJW_BranchStructure(struct CJW_BranchStructure* input);
/********************************************************/
void Print_CJW_BranchDynamicState(struct CJW_BranchDynamicState* input);
/********************************************************/
void Print_CJW_RobotStructure(struct CJW_RobotStructure* input);

/****************show the structure*********************************************************************/
void Print_CJW_RigidBody(struct CJW_RigidBody* input)
{
        printf("the rigid body parament:\n");
        printf("BodyV is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->BodyV[i]);}
        printf("]\nBodyA is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->BodyA[i]);}
        printf("]\nBodyF is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->BodyF[i]);}
        printf("]\nMG is \n");
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                printf("%.3f ",input->MG[i][j]);
            }
            printf("\n");
        }
        printf("M is \n");
        for(int i=0;i<6;i++)
        {
            for(int j=0;j<6;j++)
            {
                printf("%.3f ",input->M[i][j]);
            }
            printf("\n");
        }
}
/********************************************************/
void Print_CJW_TipStruct(struct CJW_TipStruct* input)
{
        printf("The CJW_TipStruct:\n");
        printf("]\nG is \n");
        for(int i=0;i<4;i++)
        {
            for(int j=0;j<4;j++)
            {
                printf("%.3f ",input->G[i][j]);
            }
            printf("\n");
        }
        printf("BodyV is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->V[i]);}
        printf("]\nBodyA is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->A[i]);}
        printf("]\nBodyF is [ ");
        for(int i=0;i<6;i++){printf("%.3f; ",input->F[i]);}
        printf("]\n");
}
/********************************************************/
void Print_CJW_BasedJoint(struct CJW_BasedJoint* input)
{
    printf("the CJW_BasedJoint %s is ",input->name.c_str());
    printf("Angle:%.3f; Velocity:%.3f; Accelation:%.3f; Force:%.3f;\n",input->Angle,input->Velocity,input->Accelation,input->Force);
    printf("AngleLimit:[%.3f;%.3f] VelocityLimit::[%.3f;%.3f] VelocityLimit::[%.3f;%.3f] VelocityLimit::[%.3f;%.3f] \n",
           input->AngleLimit[0],input->AngleLimit[1],input->VelocityLimit[0],input->VelocityLimit[1],
            input->AccelationLimit[0],input->AccelationLimit[1],input->ForceLimit[0],input->ForceLimit[1]);
}
/********************************************************/
void Print_CJW_JointOffset(struct CJW_JointOffset* input)
{
    printf("the offset:[%.3f;%.3f]\n",input->direction,input->AngleInit);
}
/********************************************************/
void Print_CJW_JointStruct(struct CJW_JointStruct* input)
{
    printf("the CJW_JointStruct %s is \n",input->name.c_str());
    printf("G0 is \n");
    for(int i=0;i<4;i++)
    {
        for(int j=0;j<4;j++)
        {
            printf("%.3f ",input->G0[i][j]);
        }
        printf("\n");
    }
    printf("Screw0 is [");
    for(int i=0;i<6;i++){printf("%.3f; ",input->Screw0[i]);}
    printf("]\n");
    if(input->parent!=NULL){printf("the parent is %s \n",input->parent->name.c_str());}
    if(input->Body.M[4][4])
    {
        Print_CJW_RigidBody(&(input->Body));
    }
}
/********************************************************/
void Print_CJW_BranchModel(struct CJW_BranchModel* input)
{
    printf("the Print_CJW_BranchModel %d: JointN is %d\n; State is %.1f; \n",input->modelstate,input->JointN,input->State);
    for(int i=0;i<input->JointN;i++)
    {
        Print_CJW_BasedJoint(input->Joint[i]);
        //Print_CJW_JointOffset(&(input->JointOffset[i]));
        Print_CJW_JointStruct(&(input->JStruct[i]));
    }
    Print_CJW_JointStruct(&(input->JStruct[input->JointN]));
}
/********************************************************/
void Print_CJW_BranchStructure(struct CJW_BranchStructure* input)
{
    printf("the Print_CJW_BranchModel %s: JointN is %d\n; \n",input->name.c_str(),input->BodyN);
    for(int i=0;i<input->BodyN;i++)
    {
        Print_CJW_BasedJoint(&(input->Joint[i]));
        Print_CJW_JointStruct(&(input->JStruct[i]));
    }
    for(int model=0;model<BRANCH_MODEL_MAX;model++)
    {
        if(input->Model_EN[model])
        {
            Print_CJW_BranchModel(&(input->BModel[model]));
        }
    }
}
/********************************************************/
void Print_CJW_BranchDynamicState(struct CJW_BranchDynamicState* input)
{
    printf("the Inertia is\n");
    for(int i=0;i<36;i++)
    {
        printf("%.3f ",input->Inertia[i]);
        if((i%6)==5) {printf("\n");}
    }
    printf("the dInertia is\n");
    for(int i=0;i<36;i++)
    {
        printf("%.3f ",input->dInertia[i]);
        if((i%6)==5) {printf("\n");}
    }
    printf("the Momentum is\n");
    for(int i=0;i<6;i++)
    {
        printf("%.3f ",input->Momentum[i]);
    }
    printf("\n");
}

/********************************************************/
void Print_CJW_RobotStructure(struct CJW_RobotStructure* input)
{
    printf("the CJW_RobotStructure %s: BranchN is %d\n; \n",input->name.c_str(),input->BranchN);
    Print_CJW_JointStruct(&(input->MainBody));
    for(int i=0;i<input->BranchN;i++)
    {
        Print_CJW_JointStruct(&(input->BranchP[i]));
        Print_CJW_BranchStructure(&(input->BranchBody[i]));
    }
}

