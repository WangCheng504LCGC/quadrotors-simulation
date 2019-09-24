function yout=HeightCtrl(z_E,ADRC_Height,QuadPara,x,h)%
%% 姿态角和模型参数
deg1=x(4);
deg2=x(5);
m=QuadPara.m;kcr=QuadPara.Cr/1000;wb=QuadPara.wb;Tm=QuadPara.Tm;
%% 状态量：高度和速度
z=x(12);
vz=x(9);

%% 自抗扰控制参数
ADRC_Height.h=h; 
ADRC_Height.h0=10*ADRC_Height.h; 
ADRC_Height.b0= m;
ADRC_Height.beita01 = 40;
ADRC_Height.beita02 = 800;
ADRC_Height.r0 = 1.5;
ADRC_Height.Vmax = 2;
%% 跟踪微分器
fh=fhan(ADRC_Height.v1-z_E,ADRC_Height.v2,ADRC_Height.r0,ADRC_Height.h0);
ADRC_Height.v1=ADRC_Height.v1+ADRC_Height.h*ADRC_Height.v2;
ADRC_Height.v2=ADRC_Height.v2+ADRC_Height.h*fh;
ADRC_Height.v2=LIMIT_Min_Max(ADRC_Height.v2,-ADRC_Height.Vmax,ADRC_Height.Vmax); 
%% 一阶扩张状态观测器
e=ADRC_Height.z1-vz;
ADRC_Height.z1=ADRC_Height.z1+ADRC_Height.h*(ADRC_Height.z2+ADRC_Height.b0*ADRC_Height.u0-ADRC_Height.beita01*e);
ADRC_Height.z2=ADRC_Height.z2+ADRC_Height.h*(-ADRC_Height.beita02*e);       

%% 误差反馈控制律 
kp=50;        kd=100;
e=ADRC_Height.v1-z;
e2=ADRC_Height.v2-ADRC_Height.z1;
ADRC_Height.u0=(kp*e+kd*e2-ADRC_Height.z2)/ADRC_Height.b0;
ADRC_Height.u0=LIMIT_Min_Max(ADRC_Height.u0,0,300); 
ADRC_Height.u0=ADRC_Height.u0/(cos(deg1)*cos(deg2));
usgn=sgn(ADRC_Height.u0)*sqrt(abs(ADRC_Height.u0))*180;
usgn_=sgn(ADRC_Height.last_u)*sqrt(abs(ADRC_Height.last_u))*180;
ADRC_Height.u=Tm/(kcr*h)*(usgn-usgn_)+1/kcr*usgn-(wb+350)/kcr;

ADRC_Height.last_u=ADRC_Height.u0 ;    

yout=ADRC_Height;



  