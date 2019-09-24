function [ADRC_Angle,ADRC_Aspeed]=AngCtrl(AttE,ADRC_Angle,ADRC_Aspeed,x,h)%

%% 1. 姿态角和期望角转换,模型参数转换
angle=x(4:6)*180/pi;
angle(3)=To_180_degree(angle(3));
x(6)=angle(3)*pi/180;
AttE(3)= To_180_degree(AttE(3));
angleE_Rad=AttE*pi/180;
% angleE_T(1)=sin(angleE_Rad(1));angleE_T(2)=sin(angleE_Rad(2));angleE_T(3)=angleE_Rad(3);
angleE_T(1)=angleE_Rad(1);angleE_T(2)=angleE_Rad(2);angleE_T(3)=angleE_Rad(3);
%% 三角函数值求解
Cos_Pitch=cos(angle(1)*pi/180);Sin_Pitch=sin(angle(1)*pi/180);
Cos_Roll=cos(angle(2)*pi/180);Sin_Roll=sin(angle(2)*pi/180);
Tan_Roll=tan(angle(2)*pi/180);Tan_Pitch=tan(angle(1)*pi/180);
%% 跟踪微分器
ADRC_Angle.h=h; 
ADRC_Angle.h0=5*ADRC_Angle.h; 
ADRC_Angle.r0 = 500000;
ADRC_Angle.rz = 10;
fh=fhan(ADRC_Angle.v1.x-angleE_T(1),ADRC_Angle.v2.x,ADRC_Angle.r0,ADRC_Angle.h0);
ADRC_Angle.v1.x=ADRC_Angle.v1.x+ADRC_Angle.h*ADRC_Angle.v2.x;
ADRC_Angle.v2.x=ADRC_Angle.v2.x+ADRC_Angle.h*fh;

fh=fhan(ADRC_Angle.v1.y-angleE_T(2),ADRC_Angle.v2.y,ADRC_Angle.r0,ADRC_Angle.h0);
ADRC_Angle.v1.y=ADRC_Angle.v1.y+ADRC_Angle.h*ADRC_Angle.v2.y;
ADRC_Angle.v2.y=ADRC_Angle.v2.y+ADRC_Angle.h*fh;

fh=fhan(ADRC_Angle.v1.z-angleE_T(3),ADRC_Angle.v2.z,ADRC_Angle.rz,ADRC_Angle.h0);
ADRC_Angle.v1.z=ADRC_Angle.v1.z+ADRC_Angle.h*ADRC_Angle.v2.z;
ADRC_Angle.v2.z=ADRC_Angle.v2.z+ADRC_Angle.h*fh;
%% 误差反馈控制律 
%Pitch
K23=0.4;S12=0;U4=0;m=1;S22=0;
ADRC_Angle.S23=(x(4)-ADRC_Angle.v1.x)*180/pi;
ADRC_Angle.u0.x=-K23*ADRC_Angle.S23;%(-K23*ADRC_Angle.S23-sin(x(6))*S12*U4/m+cos(x(6))*S22*U4/m)/Cos_Pitch;
ADRC_Aspeed.S23=ADRC_Angle.S23;
%Roll
K13=0.4;S12=0;U4=0;m=1;S22=0;lambda=0;
ADRC_Angle.S13=(x(5)-ADRC_Angle.v1.y)*180/pi;
ADRC_Angle.u0.y=-K13*ADRC_Angle.S13;%(-K13*ADRC_Angle.S13-sqrt(lambda)*cos(x(6))*S12*U4/m-sqrt(lambda)*sin(x(6))*S22*U4/m)/Cos_Roll;
ADRC_Aspeed.S13=ADRC_Angle.S13;

K41=0.4;
ADRC_Angle.S41=(x(6)-ADRC_Angle.v1.z)*180/pi;
ADRC_Angle.S41= To_180_degree(ADRC_Angle.S41);
ADRC_Angle.u0.z=(-K41*ADRC_Angle.S41);
ADRC_Aspeed.S41=ADRC_Angle.S41;









  