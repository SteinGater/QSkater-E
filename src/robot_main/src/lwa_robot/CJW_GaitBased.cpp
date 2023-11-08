#include "CJW_GaitBased.h"

/************************************************************************************/
int  CJW_GaitBased::GetGaitN(void)
{
    return GaitN;
}
int  CJW_GaitBased::GetBranchiModel(int legi)
{
    return GaitBranchModel[legi];
}
void CJW_GaitBased::SetBranchiModel(int legi,int model)
{
    GaitBranchModel[legi]=model;
}
void CJW_GaitBased::GetBranchModel(int *out)
{
    for(int legi=0;legi<GaitN;legi++)
    {
        out[legi]=GaitBranchModel[legi];
    }
}
void CJW_GaitBased::SetBranchModel(int *in)
{
    for(int legi=0;legi<GaitN;legi++)
    {
        GaitBranchModel[legi]=in[legi];
    }
}
int CJW_GaitBased::GetSwingOrder(int gaittype,int num,int* out)
{
    switch(num)
    {
        case 0:  return ERROR;
        default: return ERROR;
    }
}
int CJW_GaitBased::GetWheelDirection(int gaittype,double* dir)
{
    return ERROR;
}
/************************************************************************************/
//design one DoF trajectory,the initial/end position,velocity,accelation is {start[3]/end[3]}
//{T} is the total time, the t is real time,{MaxV and MaxA} is the maxmium velocity and accelation
//{out[3]} is design position,velocity and accelation
int CJW_GaitBased::Design_1DoF(int type,double start[3],double end[3],double T,double t,double out[3])
{
    return Design_1DoF(type,start,end,T,t,out,DESIGN_MAX_V0,DESIGN_MAX_A0);
}
int CJW_GaitBased::Design_1DoF(int type,double start[3],double end[3],double T,double t,double out[3],double MaxV,double MaxA)
{
    double thet=t;if(t<0){thet=0;}else if(t>T){thet=T;}
    if(type==DESIGN_LINE){return Design_1DoF_line(start,end,T,thet,out);}
    //else if(type==DESIGN_QUADRIC){return Design_1DoF_quadric(start,end,T,thet,out);}
    else if(type==DESIGN_CUBIC){return Design_1DoF_cubic(start,end,T,thet,out);}
    //else if(type==DESIGN_QUARTIC){return Design_1DoF_quartic(start,end,T,thet,out);}
    else if(type==DESIGN_QUINTIC){return Design_1DoF_quintic(start,end,T,thet,out);}
    else if(type==DESIGN_SIN){return Design_1DoF_sin(start,end,T,thet,out);}
    else if(type==DESIGN_TRAPEZIUM){return Design_1DoF_trapezium(start,end,T,thet,out,MaxV,MaxA);}
    else if(type==DESIGN_CYCLOID){return Design_1DoF_cycloid(start,end,T,thet,out);}
    else {out[0]=0;out[1]=0;out[2]=0;return ERROR;}
}
int CJW_GaitBased::Design_1DoF_line(double start[3],double end[3],double T,double t,double out[3])
{//x=fa+fbt;
    out[0]=start[0]+(end[0]-start[0])*t/T;out[1]=(end[0]-start[0])/T;out[2]=0;
    return RIGHT;
}
int CJW_GaitBased::Design_1DoF_cubic(double start[3],double end[3],double T,double t,double out[3])
{//x=fa+fb*t+fc*t^2+fd*t^3;
    double fa=start[0];double fb=start[1];
    double fc=(3*end[0]-3*fa-2*fb*T-T*end[1])/T/T;double fd=(-2*end[0]+2*fa+fb*T+T*end[1])/T/T/T;
    out[0]=fa+  fb*t+   fc*t*t+ fd*t*t*t;
    out[1]=     fb+     2*fc*t+ 3*fd*t*t;
    out[2]=             2*fc+   6*fd*t;
    return RIGHT;
}
int CJW_GaitBased::Design_1DoF_quintic(double start[3],double end[3],double T,double t,double out[3])
{//x=fa+fb*t+fc*t^2+fd*t^3+fe*t^4+ff*t^5;
    double fa=start[0];double fb=start[1];double fc=start[2]/2;
    double T2c=1.0*T*T;double T3c=T2c*T;double T4c=T3c*T;double T5c=T4c*T;
    double fd=-(20*start[0]-20*end[0]+8*T*end[1]+12*T*start[1]-T2c*end[2]+3*T2c*start[2])/2/T3c;
    double fe= (30*start[0]-30*end[0]+14*T*end[1]+16*T*start[1]-2*T2c*end[2]+3*T2c*start[2])/2/T4c;
    double ff=-(12*start[0]-12*end[0]+6*T*end[1]+6*T*start[1]-T2c*end[2]+T2c*start[2])/2/T5c;
    double t2c=t*t;double t3c=t2c*t;double t4c=t3c*t;double t5c=t4c*t;
    out[0]=fa+  fb*t+   fc*t2c+     fd*t3c+     fe*t4c+     ff*t5c;
    out[1]=     fb+     2*fc*t+     3*fd*t2c+   4*fe*t3c+   5*ff*t4c;
    out[2]=             2*fc+       6*fd*t+     12*fe*t2c+  20*ff*t3c;
    return RIGHT;
}
int CJW_GaitBased::Design_1DoF_sin(double start[3],double end[3],double T,double t,double out[3])
{//x=A*(1-cos(f*t));
    double A=(end[0]-start[0])/2;double f=M_PI/T;
    out[0]=A*(1-cos(f*t))+start[0];out[1]=A*f*sin(f*t);out[2]=A*f*f*cos(f*t);
    return RIGHT;
}
int CJW_GaitBased::Design_1DoF_trapezium(double start[3],double end[3],double T,double t,double out[3],double MaxV,double MaxA)
{//using Max
    double A=end[0]-start[0];double t0=fabs(MaxV/MaxA);
    double dir=1;if(A<0){dir=-1;}
    double Amax=dir*(fabs(MaxV)*T-fabs(MaxV*MaxV/MaxA));
    if(T<=(2*t0)){Amax=dir*fabs(MaxA*T*T/4);}
    if(fabs(Amax)<fabs(A)) return ERROR;
    double t1=(T-sqrt(T*T-4*fabs(A/MaxA)))/2;double acc=dir*fabs(MaxA);
    if(t<t1)
    {
        out[0]=start[0]+acc*t*t/2;out[1]=acc*t;out[2]=acc;
    }
    else if(t<(T-t1))
    {
        out[0]=start[0]+acc*t1*(t-t1/2);out[1]=acc*t1;out[2]=0;
    }
    else
    {
        out[0]=end[0]-acc*(T-t)*(T-t)/2;out[1]=acc*(T-t);out[2]=-acc;
    }
    return RIGHT;
}
int CJW_GaitBased::Design_1DoF_cycloid(double start[3],double end[3],double T,double t,double out[3])
{//y=r*(t-sin(t));0<t<2*pi
    double f=2*M_PI/T;
    double A=(end[0]-start[0])/2/M_PI;
    out[0]=A*(f*t-sin(f*t))+start[0];
    out[1]=A*(f-f*cos(f*t));
    out[2]=A*f*f*sin(f*t);
    return RIGHT;
}
/************************************************************************************/
//define the type of SE3 design on screw method
int CJW_GaitBased::Design_SE3(int mode,double type,double G1[4][4],double G2[4][4],double V1[6],double V2[6],double ACC1[6],double ACC2[6],
double MaxV[6],double MaxA[6],double T,double t,double Gout[4][4],double Vout[6],double Aout[6])
{
    int error=RIGHT;
    double GII[4][4]={{1,0,0,0},{0,1,0,0},{0,0,1,0},{0,0,0,1}};
    double RII[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double dG[4][4];MathFun.MyGInvCompositionG((double*)G1,(double*)G2,(double*)dG);
    double kesi1[6]={0};double dkesi1[6]={0};double ddkesi1[6]={0};
    double kesi2[6]={0};double dkesi2[6]={0};double ddkesi2[6]={0};
    double dGout[4][4];
    if(mode==DESIGN_MOVE_COUPE)//design on SE(3)
    {
        MathFun.MyGVAToExponet4((double*)GII,V1,ACC1,kesi1,dkesi1,ddkesi1);
        MathFun.MyGVAToExponet4((double*)dG ,V2,ACC2,kesi2,dkesi2,ddkesi2);
        double kesiout[6],dkesiout[6],ddkesiout[6];
        for(int ii=0;ii<6;ii++)
        {
            double s1[3]={kesi1[ii],dkesi1[ii],ddkesi1[ii]};
            double s2[3]={kesi2[ii],dkesi2[ii],ddkesi2[ii]};
            double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout,MaxV[ii],MaxA[ii]);
            if(error!=RIGHT)
            {
                return error;
            }
            kesiout[ii]=sout[0];dkesiout[ii]=sout[1];ddkesiout[ii]=sout[2];
        }
        MathFun.MyExponet4ToGVA(kesiout,dkesiout,ddkesiout,(double *)dGout,Vout,Aout);
    }
    else if(mode==DESIGN_MOVE_DECOUPE)//design on SO(3)+R3
    {
        double dR[9];double dP[3];MathFun.MyGToRP((double*)dG,dR,dP);
        MathFun.MyRVAToExponet3((double*)RII,V1,ACC1,kesi1,dkesi1,ddkesi1);
        MathFun.MyRVAToExponet3((double*)dR ,V2,ACC2,kesi2,dkesi2,ddkesi2);
        for(int ii=0;ii<3;ii++)
        {
            int ii3=ii+3;
            kesi1[ii3]=0;kesi2[ii3]=dG[ii][3];
            dkesi1[ii3]=V1[ii3];//dkesi2[ii3]=V2[ii3];
            ddkesi1[ii3]=ACC1[ii3];//ddkesi2[ii3]=ACC2[ii3];
        }
        MathFun.MyRInvCompositionw(dR,&(V2[3]),&(dkesi2[3]));
        MathFun.MyRInvCompositionw(dR,&(ACC2[3]),&(ddkesi2[3]));
        double kesiout[6],dkesiout[6],ddkesiout[6];
        for(int ii=0;ii<6;ii++)
        {
            double s1[3]={kesi1[ii],dkesi1[ii],ddkesi1[ii]};
            double s2[3]={kesi2[ii],dkesi2[ii],ddkesi2[ii]};
            double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout,MaxV[ii],MaxA[ii]);
            if(error!=RIGHT)
            {
                return error;
            }
            kesiout[ii]=sout[0];dkesiout[ii]=sout[1];ddkesiout[ii]=sout[2];
        }
        double dRout[3][3];MathFun.MyExponet3ToRVA(kesiout,dkesiout,ddkesiout,(double *)dRout,Vout,Aout);
        MathFun.MyRInvCompositionw((double*)dRout,&(dkesiout[3]),&(Vout[3]));
        MathFun.MyRInvCompositionw((double*)dRout,&(ddkesiout[3]),&(Aout[3]));
        for(int ii=0;ii<3;ii++)
        {
            for(int jj=0;jj<3;jj++)
            {
                dGout[ii][jj]=dRout[ii][jj];
            }
            dGout[ii][3]=kesiout[ii+3];
        }
        dGout[3][0]=0;dGout[3][1]=0;dGout[3][2]=0;dGout[3][3]=1;
    }
    else
    {
        return ERROR;
    }
    MathFun.MyGCompositionG((double*)G1,(double*)dGout,(double*)Gout);
    return RIGHT;
}
int CJW_GaitBased::Design_SO3(double type,double R1[3][3],double R2[3][3],double V1[3],double V2[3],double ACC1[3],double ACC2[3],
double MaxV[3],double MaxA[3],double T,double t,double Rout[3][3],double Vout[3],double Aout[3])
{
    int error=RIGHT;
    double RII[3][3]={{1,0,0},{0,1,0},{0,0,1}};
    double dR[3][3];MathFun.MyRInvCompositionR((double*)R1,(double*)R2,(double*)dR);
    double kesi1[3]={0};double dkesi1[3]={0};double ddkesi1[3]={0};
    double kesi2[3]={0};double dkesi2[3]={0};double ddkesi2[3]={0};
    MathFun.MyRVAToExponet3((double*)RII,V1,ACC1,kesi1,dkesi1,ddkesi1);
    MathFun.MyRVAToExponet3((double*)dR ,V2,ACC2,kesi2,dkesi2,ddkesi2);
    double kesiout[3],dkesiout[3],ddkesiout[3];
    for(int ii=0;ii<3;ii++)
    {
        double s1[3]={kesi1[ii],dkesi1[ii],ddkesi1[ii]};
        double s2[3]={kesi2[ii],dkesi2[ii],ddkesi2[ii]};
        double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout,MaxV[ii],MaxA[ii]);
        if(error!=RIGHT)
        {
            return error;
        }
        kesiout[ii]=sout[0];dkesiout[ii]=sout[1];ddkesiout[ii]=sout[2];
    }
    double dRout[3][3];MathFun.MyExponet3ToRVA(kesiout,dkesiout,ddkesiout,(double *)dRout,Vout,Aout);
    MathFun.MyRCompositionR((double*)R1,(double*)dRout,(double*)Rout);
    return RIGHT;
}
int CJW_GaitBased::Design_R3(double type,double P1[3],double P2[3],double V1[3],double V2[3],double ACC1[3],double ACC2[3],
double MaxV[3],double MaxA[3],double T,double t,double Pout[3],double Vout[3],double Aout[3])
{
    int error=RIGHT;
    for(int ii=0;ii<3;ii++)
    {
        double s1[3]={P1[ii],V1[ii],ACC1[ii]};
        double s2[3]={P2[ii],V2[ii],ACC2[ii]};
        double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout,MaxV[ii],MaxA[ii]);
        if(error!=RIGHT)
        {
            return error;
        }
        Pout[ii]=sout[0];Vout[ii]=sout[1];Aout[ii]=sout[2];
    }
    return RIGHT;
}
int CJW_GaitBased::Design_SE3_ZERO(int mode,double type,double G1[4][4],double G2[4][4],double T,double t,double Gout[4][4],double Vout[6],double Aout[6])
{
    int error=RIGHT;
    //design into one DoF
    double s1[3]={0,0,0};double s2[3]={1.0,0,0};
    double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout);
    if(error!=RIGHT)
    {
        return error;
    }
    //using to the SE(3)
    double dG[4][4];MathFun.MyGInvCompositionG((double*)G1,(double*)G2,(double*)dG);
    double kesi0[6]={0};
    double dGout[4][4];
    if(mode==DESIGN_MOVE_COUPE)//design on SE(3)
    {
        MathFun.MyGToExponent4((double*)dG ,kesi0);
        double kesiout[6],dkesiout[6],ddkesiout[6];
        for(int ii=0;ii<6;ii++)
        {
            kesiout[ii]=kesi0[ii]*sout[0];dkesiout[ii]=kesi0[ii]*sout[1];ddkesiout[ii]=kesi0[ii]*sout[2];
        }
        MathFun.MyExponet4ToGVA(kesiout,dkesiout,ddkesiout,(double *)dGout,Vout,Aout);
    }
    else if(mode==DESIGN_MOVE_DECOUPE)//design on SO(3)+R3
    {
        double dR[9];double dP[3];MathFun.MyGToRP((double*)dG,dR,dP);
        MathFun.MyRToExponent3(dR,kesi0);
        kesi0[3]=dG[0][3];kesi0[4]=dG[1][3];kesi0[5]=dG[2][3];
        double kesiout[6],dkesiout[6],ddkesiout[6];
        for(int ii=0;ii<6;ii++)
        {
            kesiout[ii]=kesi0[ii]*sout[0];dkesiout[ii]=kesi0[ii]*sout[1];ddkesiout[ii]=kesi0[ii]*sout[2];
        }
        double dRout[3][3];MathFun.MyExponet3ToRVA(kesiout,dkesiout,ddkesiout,(double *)dRout,Vout,Aout);
        MathFun.MyRInvCompositionw((double*)dRout,&(dkesiout[3]),&(Vout[3]));
        MathFun.MyRInvCompositionw((double*)dRout,&(ddkesiout[3]),&(Aout[3]));
        for(int ii=0;ii<3;ii++)
        {
            for(int jj=0;jj<3;jj++)
            {
                dGout[ii][jj]=dRout[ii][jj];
            }
            dGout[ii][3]=kesiout[ii+3];
        }
        dGout[3][0]=0;dGout[3][1]=0;dGout[3][2]=0;dGout[3][3]=1;
    }
    else
    {
        return ERROR;
    }
    MathFun.MyGCompositionG((double*)G1,(double*)dGout,(double*)Gout);
    return RIGHT;
}
int CJW_GaitBased::Design_SO3_ZERO(double type,double R1[3][3],double R2[3][3],double T,double t,double Rout[3][3],double Vout[3],double Aout[3])
{
    int error=RIGHT;
    //design into one DoF
    double s1[3]={0,0,0};double s2[3]={1.0,0,0};
    double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout);
    if(error!=RIGHT)
    {
        return error;
    }
    //using to the SO(3)
    double dR[3][3];MathFun.MyRInvCompositionR((double*)R1,(double*)R2,(double*)dR);
    double kesi0[3]={0};MathFun.MyRToExponent3((double*)dR ,kesi0);
    double kesiout[3],dkesiout[3],ddkesiout[3];
    for(int ii=0;ii<3;ii++)
    {
        kesiout[ii]=kesi0[ii]*sout[0];dkesiout[ii]=kesi0[ii]*sout[1];ddkesiout[ii]=kesi0[ii]*sout[2];
    }
    double dRout[3][3];MathFun.MyExponet3ToRVA(kesiout,dkesiout,ddkesiout,(double *)dRout,Vout,Aout);
    MathFun.MyRCompositionR((double*)R1,(double*)dRout,(double*)Rout);
    return RIGHT;
}
int CJW_GaitBased::Design_R3_ZERO(double type,double P1[3],double P2[3],double T,double t,double Pout[3],double Vout[3],double Aout[3])
{
    int error=RIGHT;
    //design into one DoF
    double s1[3]={0,0,0};double s2[3]={1.0,0,0};
    double sout[3];error=Design_1DoF(type,s1,s2,T,t,sout);
    if(error!=RIGHT)
    {
        return error;
    }
    for(int ii=0;ii<3;ii++)
    {
        double theA=P2[ii]-P1[ii];
        Pout[ii]=theA*sout[0]+P1[ii];Vout[ii]=theA*sout[1];Aout[ii]=theA*sout[2];
    }
    return RIGHT;
}
/************************************************************************************/
//dir:运动方向和X正方向夹角（弧度制），每个周期线步长，每个周期角度步长
int CJW_GaitBased::Design_SE2(int mode,double dir[3],double time,double outG[4][4])
{
    double angle=0;double x=0;double y=0;
    angle=dir[2]*time;
    if((mode==DESIGN_MOVE_COUPE)&&(fabs(dir[2])>0))//the xy is a circle on SE2
    {
        double theR=1.0*dir[1]/dir[2];
        x=theR*(sin(dir[0]+angle)-sin(dir[0]));
        y=theR*(-cos(dir[0]+angle)+cos(dir[0]));
    }
    else//the xy is a line
    {
        x=dir[1]*cos(dir[0])*time;
        y=dir[1]*sin(dir[0])*time;
    }
    outG[0][0]=cos(angle);outG[0][1]=-sin(angle);outG[0][2]=0;outG[0][3]=x;
    outG[1][0]=sin(angle);outG[1][1]=cos(angle); outG[1][2]=0;outG[1][3]=y;
    outG[2][0]=0;         outG[2][1]=0;          outG[2][2]=1;outG[2][3]=0;
    outG[3][0]=0;         outG[3][1]=0;          outG[3][2]=0;outG[3][3]=1;

    return RIGHT;
}
/************************************************************************************/
//define the type of one swing foot design;
int CJW_GaitBased::Design_swingfoot(int type,double start[3], double end[3], double T, double H, double time, double outP[3],double outV[3],double outA[3])
{
    double dP[3]={end[0]-start[0],end[1]-start[1],end[2]-start[2]};
    double dL[2]={sqrt(dP[0]*dP[0]+dP[1]*dP[1]),dP[2]};
    double theH=fmax(0,dL[1])+fabs(H);
    double outdP[3]={0};double outdV[3]={0};double outdA[3]={0};
    double thet=time;if(thet<0){thet=0;}else if(thet>T){thet=T;}
    if(H<=MATH_ZERO)                
    {
        Design_R3_ZERO(type,start,end,T,time,outP,outV,outA);
    }
    else 
    {
        if(type==DESIGN_SIN)            {Design_swingfoot_sin(      dL,T,H,thet,outdP,outdV,outdA);}
        else if(type==DESIGN_TRAPEZIUM) {Design_swingfoot_trapezium(dL,T,H,thet,outdP,outdV,outdA);}
        //else if(type==DESIGN_QUADRIC)   {Design_swingfoot_quadric(dL,T,H,thet,outdP,outdV,outdA);}
        else if(type==DESIGN_CYCLOID)   {Design_swingfoot_cycloid(  dL,T,H,thet,outdP,outdV,outdA);}
        else{return ERROR;}
        if(dL[0]<MATH_ZERO)
        {
            outP[0]=start[0];outP[1]=start[1];outP[2]=start[2]+outdP[1];
            outV[0]=0;outV[1]=0;outV[2]=outdV[1];
            outA[0]=0;outA[1]=0;outA[2]=outdA[1];
        }
        else
        {
            double kx=dP[0]/dL[0];double ky=dP[1]/dL[0];
            outP[0]=start[0]+outdP[0]*kx;outP[1]=start[1]+outdP[0]*ky;outP[2]=start[2]+outdP[1];
            outV[0]=outdV[0]*kx;outV[1]=outdV[0]*ky;outV[2]=outdV[1];
            outA[0]=outdA[0]*kx;outA[1]=outdA[0]*ky;outA[2]=outdA[1];
        }
    }
    return RIGHT;
}
void CJW_GaitBased::Design_swingfoot_sin(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2])
{
    double ff=M_PI/T;double A=dL[0]/2;double thet=time;
    outP[0]=A*(1-cos(ff*thet));outV[0]=A*sin(ff*thet)*ff;outA[0]=A*cos(ff*thet)*ff*ff;
    double TotalL=H+H-dL[1];double T1=H/TotalL*T;double T2=T-T1;
    if(time<T1)
    {
        ff=M_PI/T1;A=H/2;thet=time;
        outP[1]=A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
    else
    {
        ff=M_PI/T2;A=(dL[1]-H)/2;thet=time-T1;
        outP[1]=H+A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
}

void CJW_GaitBased::Design_swingfoot_trapezium(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2])
{
    double TotalL=H+dL[0]+fabs(H-dL[1]);
    double T1=H/TotalL*T;double T2=dL[0]/TotalL*T;double T3=(H-dL[1])/TotalL*T;
    if(time<=T1)
    {
        outP[0]=0;outV[0]=0;outA[0]=0;
        double ff=M_PI/T1;double A=H/2;double thet=time;
        outP[1]=A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
    else if(time<(T1+T2))
    {
        double ff=M_PI/T2;double A=dL[0]/2;double thet=time-T1;
        outP[0]=A*(1-cos(ff*thet));outV[0]=A*sin(ff*thet)*ff;outA[0]=A*cos(ff*thet)*ff*ff;
        outP[1]=H;outV[1]=0;outA[1]=0;
    }
    else
    {
        outP[0]=dL[0];outV[0]=0;outA[0]=0;
        double ff=M_PI/T3;double A=(dL[1]-H)/2;double thet=time-T1-T2;
        outP[1]=H+A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
}
void CJW_GaitBased::Design_swingfoot_cycloid(double dL[2], double T, double H, double time, double outP[2],double outV[2],double outA[2])
{//x=r(1-cos(t));y=r*(t-sin(t));0<t<2*pi
    double ff=2*M_PI/T;double thet=time;
    double A=dL[0]/2/M_PI;
    outP[0]=A*(thet*ff-sin(ff*thet));outV[0]=A*(ff-cos(ff*thet)*ff);outA[0]=-A*sin(ff*thet)*ff*ff;
    double TotalL=H+H-dL[1];double T1=H/TotalL*T;double T2=T-T1;
    if(time<T1)
    {
        ff=M_PI/T1;A=H/2;thet=time;
        outP[1]=A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
    else
    {
        ff=M_PI/T2;A=(dL[1]-H)/2;thet=time-T1;
        outP[1]=H+A*(1-cos(ff*thet));outV[1]=A*sin(ff*thet)*ff;outA[1]=A*cos(ff*thet)*ff*ff;
    }
}
/************************************************************************************/



/*************************************************************************************************************/
/*************************************************************************************************************/
/****************************the body robot gait desing for the cycle gait without control******************************/
//平面路径规划dir_mode参数如下：运动方向和X正方向夹角（弧度制），每个周期线步长，每个周期角度步长
//the OutTip V is not the TipV !!!!
int CJW_GaitBased::Design_Gait_Cycle_Based(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
int Movemode,int Bodytype,int Swingtype, double* SwingStart,double* SwingDuty,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    int error=RIGHT;
    struct CJW_JointStruct TheBody=Body0[0];double G0Inv[4][4];MathFun.MyGInv((double*)(Body0[0].G0),(double*)G0Inv);
    struct CJW_TipStruct TheFoot[BRANCH_N_MAX];double Thestate[BRANCH_N_MAX];
    for(int legi=0;legi<GaitN;legi++)
    {
        GaitBranchModel[legi]=BRANCH_LEG_MODEL;
        TheFoot[legi]=Tip0[legi];
        Thestate[legi]=BRANCH_SUP_STATE;
    }
    //find the end of body and foot
    double BodyEndDG[4][4];Design_SE2(Movemode,dir,1,BodyEndDG);
    double BodyEndG[4][4];MathFun.MyGCompositionG((double*)(Body0[0].G0),(double*)BodyEndDG,(double*)BodyEndG);
    double BodyGTemp[4][4];MathFun.MyGCompositionG((double*)BodyEndG,(double*)G0Inv,(double*)BodyGTemp);
    struct CJW_TipStruct EndFoot[BRANCH_N_MAX];
    for(int legi=0;legi<GaitN;legi++)
    {
        MathFun.MyGCompositionG((double*)BodyGTemp,(double*)(Tip0[legi].G),(double*)(EndFoot[legi].G));
    }
    //printf("the foot end position: %.3f+%.3f+%.3f\n",EndFoot[0].G[0][3],EndFoot[0].G[1][3],EndFoot[0].G[2][3]);
    //desin now state of the trajectory
    double thet=time;
    if(time<=0)
    {
        thet=0;
    }
    else if(time>T0)
    {
        thet=T0;
    }
    //design now main body
    error=error+Design_SE3_ZERO(Movemode,Bodytype,Body0[0].G0,BodyEndG,T0,thet,TheBody.G0,TheBody.Body.BodyV,TheBody.Body.BodyA);
    //design the foot
    for(int legi=0;legi<GaitN;legi++)
    {
        double foot_T0=SwingDuty[legi]*T0;
        double foot_t=(thet-SwingStart[legi]*T0);
        double StartP[3]={Tip0[legi].G[0][3],Tip0[legi].G[1][3],Tip0[legi].G[2][3]};
        double EndP[3]={EndFoot[legi].G[0][3],EndFoot[legi].G[1][3],EndFoot[legi].G[2][3]};
        double NowP[3];double NowV[3];double NowA[3];
        error=error+Design_swingfoot(Swingtype,StartP,EndP,foot_T0,StepH,foot_t,NowP,NowV,NowA);
        for(int ii=0;ii<3;ii++)
        {
            TheFoot[legi].G[ii][3]=NowP[ii];
            TheFoot[legi].V[ii+3]=NowV[ii];
            TheFoot[legi].A[ii+3]=NowA[ii];
        }
        if((foot_t>0)&&(foot_t<foot_T0)) 
        {
            Thestate[legi]=BRANCH_SWI_STATE;
        }
    }

    OutBody[0]=TheBody;
    for(int legi=0;legi<GaitN;legi++)
    {
        OutTip[legi]=TheFoot[legi];
        outstate[legi]=Thestate[legi];
    }
    return error;
}
//the based coordinate is the floating coordinate for roller-skating gait
//Step[0] is the Step Height ; Step[1] is the Step Length;Margins[0] is the body move length ; Margins[1] is the body orientation along X axis;
//the OutTip V is not the TipV !!!!
int CJW_GaitBased::Design_Skating_Cycle_Based(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,
double Margins[2],double Step[2],double T0,double time,
int Movemode,int Bodytype,int Swingtype, double SwingStart,int* Swingflag,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    int error=RIGHT;
    struct CJW_JointStruct TheBody=Body0[0];
    struct CJW_TipStruct TheFoot[BRANCH_N_MAX];double Thestate[BRANCH_N_MAX];
    for(int legi=0;legi<GaitN;legi++)
    {
        GaitBranchModel[legi]=BRANCH_WHEEL_MODEL;
        TheFoot[legi]=Tip0[legi];
        Thestate[legi]=BRANCH_SUP_STATE;
    }
    //find the end of body and foot
    double BodyEndDG[4][4]={1,0,0,0,
                            0,cos(Margins[1]),-sin(Margins[1]),Margins[0],
                            0,sin(Margins[1]), cos(Margins[1]),0,
                            0,0,0,1};
    double BodyEndG[4][4];MathFun.MyGCompositionG((double*)(Body0[0].G0),(double*)BodyEndDG,(double*)BodyEndG);
    struct CJW_TipStruct EndFoot[BRANCH_N_MAX];
    for(int legi=0;legi<GaitN;legi++)
    {
        EndFoot[legi]=Tip0[legi];
        if(Swingflag[legi]==DESIGN_SWI_STATE)
        {
            EndFoot[legi].G[0][3]=Tip0[legi].G[0][3]+Step[1]*Body0->G0[0][1];
            EndFoot[legi].G[1][3]=Tip0[legi].G[1][3]+Step[1]*Body0->G0[1][1];
        }
    }
    //printf("the foot end position: %.3f+%.3f+%.3f\n",EndFoot[0].G[0][3],EndFoot[0].G[1][3],EndFoot[0].G[2][3]);
    //desin now state of the trajectory
    double T1=T0*SwingStart;double T2=T0-T1;
    if(time<=0)
    {
        //Inial
    }
    else if(time<=T1)
    {
        double thet=time;
        //design now main body
        error=error+Design_SE3_ZERO(Movemode,Bodytype,Body0[0].G0,BodyEndG,T1,thet,TheBody.G0,TheBody.Body.BodyV,TheBody.Body.BodyA);
        //design the foot
        for(int legi=0;legi<GaitN;legi++)
        {
            if(Swingflag[legi]==DESIGN_SWI_STATE)
            {
                double StartP[3]={Tip0[legi].G[0][3],Tip0[legi].G[1][3],Tip0[legi].G[2][3]};
                double EndP[3]={EndFoot[legi].G[0][3],EndFoot[legi].G[1][3],EndFoot[legi].G[2][3]};
                double NowP[3];double NowV[3];double NowA[3];
                error=error+Design_R3_ZERO(Swingtype,StartP,EndP,T1,thet,NowP,NowV,NowA);
                for(int ii=0;ii<3;ii++)
                {
                    TheFoot[legi].G[ii][3]=NowP[ii];
                    TheFoot[legi].V[ii+3]=NowV[ii];
                    TheFoot[legi].A[ii+3]=NowA[ii];
                }
                Thestate[legi]=BRANCH_SUP_STATE+(BRANCH_BOUNDING_STATE-BRANCH_SUP_STATE)*thet/T1;
            }
        }
    }
    else if(time<T0)
    {
        double thet=time-T1;
        //design now main body
        error=error+Design_SE3_ZERO(Movemode,Bodytype,BodyEndG,Body0[0].G0,T2,thet,TheBody.G0,TheBody.Body.BodyV,TheBody.Body.BodyA);
        //design the foot
        for(int legi=0;legi<GaitN;legi++)
        {
            if(Swingflag[legi]==DESIGN_SWI_STATE)
            {
                double StartP[3]={Tip0[legi].G[0][3],Tip0[legi].G[1][3],Tip0[legi].G[2][3]};
                double EndP[3]={EndFoot[legi].G[0][3],EndFoot[legi].G[1][3],EndFoot[legi].G[2][3]};
                double NowP[3];double NowV[3];double NowA[3];
                error=error+Design_swingfoot(Swingtype,EndP,StartP,T2,Step[0],thet,NowP,NowV,NowA);
                for(int ii=0;ii<3;ii++)
                {
                    TheFoot[legi].G[ii][3]=NowP[ii];
                    TheFoot[legi].V[ii+3]=NowV[ii];
                    TheFoot[legi].A[ii+3]=NowA[ii];
                }
                Thestate[legi]=BRANCH_SWI_STATE;
            }
        }
    }
    else
    {
        //Inial
    }
    OutBody[0]=TheBody;
    for(int legi=0;legi<GaitN;legi++)
    {
        OutTip[legi]=TheFoot[legi];
        outstate[legi]=Thestate[legi];
    }
    return error;
}

/**************************** gait for typical position legged gait***********************************************/
/*int CJW_GaitBased::Design_Leg_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0, double dir[3], double StepH,double T0,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}*/
/**************************** gait for typical position arm design***********************************************/
/*int CJW_GaitBased::Design_Arm_Position0(int* arm_flag,struct CJW_TipStruct* Start, struct CJW_TipStruct* End,double T0,double time,int Movemode,
                          struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}*/
/**************************** gait for typical position wheel design***********************************************/
/*int CJW_GaitBased::Design_Wheel_Position0(struct CJW_JointStruct* BodyStart, struct CJW_JointStruct* BodyEnd,double T0,double time,int Movemode,
                            struct CJW_JointStruct* OutTip,double* outstate)
{
    return ERROR;
}*/
/*int CJW_GaitBased::Design_Skating_Position0(struct CJW_JointStruct* Body0, struct CJW_TipStruct* Tip0,double Margins[2],double Step[2],double T0,double duty,double time,
int Movemode,int Bodytype,int GaitType,int Swingtype,
struct CJW_JointStruct* OutBody,struct CJW_TipStruct* OutTip,double* outstate)
{
    return ERROR;
}*/
