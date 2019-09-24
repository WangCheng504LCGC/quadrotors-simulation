function [ADRC_Aspeed]=AngSpeedCtrl(QuadPara,ADRC_Angle,ADRC_Aspeed,x,T)%
%% 1. 姿态角和机体角速度,模型参数转换
gyro.x=x(1)*180/pi;
gyro.y=x(2)*180/pi;
gyro.z=x(3)*180/pi;

a1=QuadPara.a1;
a2=QuadPara.a2;
a3=QuadPara.a3;
b1=QuadPara.b1;
b2=QuadPara.b2;
b3=QuadPara.b3;
Tm=QuadPara.Tm;
s1=(980)/1000*QuadPara.Cr+QuadPara.wb;
s2=0.002*QuadPara.Cr;
k2=s1*s2;
k1=sqrt(2)*QuadPara.b*QuadPara.l;
k1z=2*QuadPara.d;
%% 期望角速度计算
    MAX_CTRL_ASPEED=300;MAX_CTRL_YAW_SPEED=200;
    except_AS.x = LIMIT(10*ADRC_Angle.u0.x, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
    except_AS.y = LIMIT(10*ADRC_Angle.u0.y, -MAX_CTRL_ASPEED,MAX_CTRL_ASPEED );
    except_AS.z = LIMIT(10*ADRC_Angle.u0.z, -MAX_CTRL_YAW_SPEED,MAX_CTRL_YAW_SPEED );
% %% v机体角速度%
%     x24 =gyro.x;
%     x14 =gyro.y;
%     x42 =gyro.z;%(gyro.z*Cos_Pitch+gyro.y*Sin_Pitch)/1;
% %     fai=x(4);sita=x(5);psai=x(6);
% %     x13=sin(sita); x23=sin(fai);  x41=psai;
% %% 自抗扰控制参数
    ADRC_Aspeed.h=T; 
    ADRC_Aspeed.h0=5.5*ADRC_Aspeed.h; 
    ADRC_Aspeed.r0 = 1000000;
%     ADRC_Aspeed.beita01= 280;
%     ADRC_Aspeed.beita02=3000;
    ADRC_Aspeed.beita01= 200;
    ADRC_Aspeed.beita02=5000;
% 
%     
    ADRC_Aspeed.rz = 500;
    ADRC_Aspeed.beita0z1=40;
    ADRC_Aspeed.beita0z2=400;            
%% 跟踪微分器
    fh=fhan(ADRC_Aspeed.v1.x-except_AS.x ,ADRC_Aspeed.v2.x,ADRC_Aspeed.r0,ADRC_Aspeed.h0);
    ADRC_Aspeed.v1.x=ADRC_Aspeed.v1.x+ADRC_Aspeed.h*ADRC_Aspeed.v2.x;
    ADRC_Aspeed.v2.x=ADRC_Aspeed.v2.x+ADRC_Aspeed.h*fh;
    
    fh=fhan(ADRC_Aspeed.v1.y-except_AS.y ,ADRC_Aspeed.v2.y,ADRC_Aspeed.r0,ADRC_Aspeed.h0);
    ADRC_Aspeed.v1.y=ADRC_Aspeed.v1.y+ADRC_Aspeed.h*ADRC_Aspeed.v2.y;
    ADRC_Aspeed.v2.y=ADRC_Aspeed.v2.y+ADRC_Aspeed.h*fh;
    
    fh=fhan(ADRC_Aspeed.v1.z-except_AS.z ,ADRC_Aspeed.v2.z,ADRC_Aspeed.rz,ADRC_Aspeed.h0);
    ADRC_Aspeed.v1.z=ADRC_Aspeed.v1.z+ADRC_Aspeed.h*ADRC_Aspeed.v2.z;
    ADRC_Aspeed.v2.z=ADRC_Aspeed.v2.z+ADRC_Aspeed.h*fh;
% %% 扩张状态观测器估计干扰量
%     e=ADRC_Aspeed.z1.x-x24;
%     ADRC_Aspeed.z1.x=ADRC_Aspeed.z1.x+ADRC_Aspeed.h*(ADRC_Aspeed.z2.x+b1*ADRC_Aspeed.u0.x-ADRC_Aspeed.beita01*e);
%     ADRC_Aspeed.z2.x=ADRC_Aspeed.z2.x+ADRC_Aspeed.h*(-ADRC_Aspeed.beita02*e);   
%     ADRC_Aspeed.S24=ADRC_Aspeed.z1.x-ADRC_Aspeed.v1.x;
%     
%     e=ADRC_Aspeed.z1.y-x14;
%     ADRC_Aspeed.z1.y=ADRC_Aspeed.z1.y+ADRC_Aspeed.h*(ADRC_Aspeed.z2.y+b2*ADRC_Aspeed.u0.y-ADRC_Aspeed.beita01*e);
%     ADRC_Aspeed.z2.y=ADRC_Aspeed.z2.y+ADRC_Aspeed.h*(-ADRC_Aspeed.beita02*e);   
%     ADRC_Aspeed.S14=x14-ADRC_Aspeed.v1.y;
%     
%     e=ADRC_Aspeed.z1.z-x42;
%     ADRC_Aspeed.z1.z=ADRC_Aspeed.z1.z+ADRC_Aspeed.h*(ADRC_Aspeed.z2.z+b3*ADRC_Aspeed.u0.z-ADRC_Aspeed.beita0z1*e);
%     ADRC_Aspeed.z2.z=ADRC_Aspeed.z2.z+ADRC_Aspeed.h*(-ADRC_Aspeed.beita0z2*e);   
%     ADRC_Aspeed.S42=x42-ADRC_Aspeed.v1.z;
%% 误差反馈控制律 
    Kpx=1;     Kpy=2;     Kpz=20;          INT_LIMIT_X=320;
    Kix=0.0;   Kiy=0.5;   Kiz=0.0;         INT_LIMIT_Y=320;    
    Kdx=2;     Kdy=2;     Kdz=3;           INT_LIMIT_Z=280;
     %/* 角速度直接微分（角加速度），负反馈可形成角速度的阻尼（阻碍角速度的变化）*/
    ADRC_Aspeed.temp.x = (gyro.x-ADRC_Aspeed.gyro_old.x) *( 0.002/T );
    ADRC_Aspeed.temp.y = (gyro.y-ADRC_Aspeed.gyro_old.y) *( 0.002/T );
    ADRC_Aspeed.temp.z = (gyro.z-ADRC_Aspeed.gyro_old.z) *( 0.002/T );
    % /* 角速度偏差 */
    ADRC_Aspeed.err.x =  (except_AS.x - gyro.x ) ;
    ADRC_Aspeed.err.y =  (except_AS.y - gyro.y ) ;
    ADRC_Aspeed.err.z =  (except_AS.z - gyro.z ) ;    
   % /* 角速度偏差权重 */
    ADRC_Aspeed.err_weight.x = ABS(ADRC_Aspeed.err.x)/MAX_CTRL_ASPEED;
    ADRC_Aspeed.err_weight.y = ABS(ADRC_Aspeed.err.y)/MAX_CTRL_ASPEED;
    ADRC_Aspeed.err_weight.z = ABS(ADRC_Aspeed.err.z)/MAX_CTRL_YAW_SPEED;
   % /* 角速度微分 */
    ADRC_Aspeed.err_d.x = Kdx *( -10 *ADRC_Aspeed.temp.x) *( 0.002/T ) ;
    ADRC_Aspeed.err_d.y = Kdy*( -10 *ADRC_Aspeed.temp.y) *( 0.002/T ) ;
    ADRC_Aspeed.err_d.z = Kdz *( -10 *ADRC_Aspeed.temp.z) *( 0.002/T) ;
   % /* 角速度误差积分*/
    ADRC_Aspeed.err_i.x = ADRC_Aspeed.err_i.x + Kix *(ADRC_Aspeed.err.x - ADRC_Aspeed.temp.x) *T;
    ADRC_Aspeed.err_i.y = ADRC_Aspeed.err_i.y + Kiy *(ADRC_Aspeed.err.y - ADRC_Aspeed.temp.y) *T;
    ADRC_Aspeed.err_i.z = ADRC_Aspeed.err_i.z + Kiz *(ADRC_Aspeed.err.z - ADRC_Aspeed.temp.z) *T;
   %  /* 角速度误差积分限幅 */
    ADRC_Aspeed.err_i.x = LIMIT( ADRC_Aspeed.err_i.x, -INT_LIMIT_X,INT_LIMIT_X);
    ADRC_Aspeed.err_i.y = LIMIT( ADRC_Aspeed.err_i.y, -INT_LIMIT_Y,INT_LIMIT_Y );
    ADRC_Aspeed.err_i.z = LIMIT( ADRC_Aspeed.err_i.z, -INT_LIMIT_Z,INT_LIMIT_Z);
   %  /* 角速度PID输出 */
    ADRC_Aspeed.u3.x = 3 *( 0.2*LIMIT((0.45 + 0.55*ADRC_Aspeed.err_weight.x),0,1)*except_AS.x + 0.8*Kpx*( ADRC_Aspeed.err.x + ADRC_Aspeed.err_d.x + ADRC_Aspeed.err_i.x ) );
    ADRC_Aspeed.u3.y = 3 *( 0.2*LIMIT((0.45 + 0.55*ADRC_Aspeed.err_weight.y),0,1)*except_AS.y + 0.8*Kpy*( ADRC_Aspeed.err.y + ADRC_Aspeed.err_d.y + ADRC_Aspeed.err_i.y ) );
    ADRC_Aspeed.u3.z = 3 *( 0.2*LIMIT((0.45 + 0.55*ADRC_Aspeed.err_weight.z),0,1)*except_AS.z + 0.8*Kpz*( ADRC_Aspeed.err.z + ADRC_Aspeed.err_d.z + ADRC_Aspeed.err_i.z ) );
   %  /* 保存上次的值 */  
    ADRC_Aspeed.gyro_old.x=  gyro.x ;
    ADRC_Aspeed.gyro_old.y=  gyro.y ;
    ADRC_Aspeed.gyro_old.z=  gyro.z ;    
%  
%     num=1.0e-07 *[  0   7.8005  7.6972];den =[ 1.0000   -1.9604    0.9608];
%     ADRC_Aspeed.T3.y=-den(2)*ADRC_Aspeed.T2.y-den(3)*ADRC_Aspeed.T1.y+num(2)*ADRC_Aspeed.u3.y+num(3)*ADRC_Aspeed.u2.y;
%     ADRC_Aspeed.u1.y=ADRC_Aspeed.u2.y;
%     ADRC_Aspeed.u2.y=ADRC_Aspeed.u3.y;
%     ADRC_Aspeed.T1.y=ADRC_Aspeed.T2.y;
%     ADRC_Aspeed.T2.y=ADRC_Aspeed.T3.y;
%     
%     e=ADRC_Aspeed.z1.y-x(2);
%     ADRC_Aspeed.z1.y=ADRC_Aspeed.z1.y+ADRC_Aspeed.h*(ADRC_Aspeed.z2.y+b2*ADRC_Aspeed.T0.y-ADRC_Aspeed.beita01*e);
%     ADRC_Aspeed.z2.y=ADRC_Aspeed.z2.y+ADRC_Aspeed.h*(-ADRC_Aspeed.beita02*e);   
% %    ADRC_Aspeed.S14=x14-ADRC_Aspeed.v1.y;   
%     
%     ADRC_Aspeed.T0.y=(ADRC_Aspeed.T3.y*b2)/b2;
%     ADRC_Aspeed.u3.y=1/num(2)*(den(1)*ADRC_Aspeed.T0.y+den(2)*ADRC_Aspeed.F1.y +den(3)*ADRC_Aspeed.F0.y-num(3)*ADRC_Aspeed.F3.y);
%     ADRC_Aspeed.F0.y=ADRC_Aspeed.F1.y;
%     ADRC_Aspeed.F1.y=ADRC_Aspeed.T0.y;
%     ADRC_Aspeed.F2.y=ADRC_Aspeed.F3.y;
%     ADRC_Aspeed.F3.y=ADRC_Aspeed.u3.y;
    %     K24=45;K24i=10;
%    % if abs(ADRC_Aspeed.S24<1150) -ADRC_Aspeed.z2.y
%       ADRC_Aspeed.ui.x=ADRC_Aspeed.ui.x-K24i*ADRC_Aspeed.h*ADRC_Aspeed.S24;
%       ADRC_Aspeed.ui.x=LIMIT_Min_Max(ADRC_Aspeed.ui.x,-200,200); 
%    % end   
%     ADRC_Aspeed.u0.x=1/b1*(ADRC_Aspeed.ui.x-x42*x14*a1+2*ADRC_Aspeed.v2.x-K24*ADRC_Aspeed.S24+1*sqrt(1-x23^2)*ADRC_Aspeed.S23-ADRC_Aspeed.z2.x);
%     ADRC_Aspeed.u0.x=LIMIT_Min_Max(ADRC_Aspeed.u0.x,-2,2); 
%     ADRC_Aspeed.u1.x=sgn(ADRC_Aspeed.u0.x)*sqrt(abs(ADRC_Aspeed.u0.x));
%     ADRC_Aspeed.u3.x=1*Tm*(ADRC_Aspeed.u1.x-ADRC_Aspeed.u2.x)/T+ADRC_Aspeed.u1.x;
%     ADRC_Aspeed.u2.x=ADRC_Aspeed.u1.x;
%     ADRC_Aspeed.u3.x=sgn(ADRC_Aspeed.u3.x)*(ADRC_Aspeed.u3.x^2)/(k1*k2);  
%     
%     K14=15;K14i=10;
%   %  if abs(ADRC_Aspeed.S14<1130)
%       ADRC_Aspeed.ui.y=ADRC_Aspeed.ui.y-K14i*ADRC_Aspeed.h*ADRC_Aspeed.S14;
%       ADRC_Aspeed.ui.y=LIMIT_Min_Max(ADRC_Aspeed.ui.y,-200,200); 
%    % end    
%     ADRC_Aspeed.u0.y=(-K14*ADRC_Aspeed.S14);    
% %     ADRC_Aspeed.u0.y=LIMIT_Min_Max(ADRC_Aspeed.u0.y,-2,2);   
% %     ADRC_Aspeed.u1.y=sgn(ADRC_Aspeed.u0.y)*sqrt(abs(ADRC_Aspeed.u0.y));
% %     ADRC_Aspeed.u3.y=1*Tm*(ADRC_Aspeed.u1.y-ADRC_Aspeed.u2.y)/T+ADRC_Aspeed.u1.y;
% %     ADRC_Aspeed.u2.y=ADRC_Aspeed.u1.y;
%     ADRC_Aspeed.u3.y= ADRC_Aspeed.u0.y;%0.5*sgn(ADRC_Aspeed.u3.y)*(ADRC_Aspeed.u3.y^2)/(k1*k2);    
% 
%     K42=100;K42i=10;
%     %if abs(ADRC_Aspeed.S42<1130)
%       ADRC_Aspeed.ui.z=ADRC_Aspeed.ui.z-K42i*ADRC_Aspeed.h*ADRC_Aspeed.S42;
%       ADRC_Aspeed.ui.z=LIMIT_Min_Max(ADRC_Aspeed.ui.z,-200,200); 
%    % end
%     ADRC_Aspeed.u0.z=1/b3*(ADRC_Aspeed.ui.z-x24*x14*a3+2*ADRC_Aspeed.v2.z-K42*ADRC_Aspeed.S42-ADRC_Aspeed.S41-ADRC_Aspeed.z2.z);%-ADRC_Aspeed.z2.z
%     ADRC_Aspeed.u1.z=sgn(ADRC_Aspeed.u0.z)*sqrt(abs(ADRC_Aspeed.u0.z));
%     ADRC_Aspeed.u3.z=1*Tm*(ADRC_Aspeed.u1.z-ADRC_Aspeed.u2.z)/T+ADRC_Aspeed.u1.z;
%     ADRC_Aspeed.u2.z=ADRC_Aspeed.u1.z;
%     ADRC_Aspeed.u3.z=sgn(ADRC_Aspeed.u3.z)*(ADRC_Aspeed.u3.z^2)/(k1z*k2);    
%     
%     
%     
% 
%     % /* 角速度误差积分 */
%     ctrl_angular_velocity.err_i.x += ctrl_angular_velocity.PID[PIDROLL].ki  *(ctrl_angular_velocity.err.x - ctrl_angular_velocity.damp.x) *T;
%     ctrl_angular_velocity.err_i.y += ctrl_angular_velocity.PID[PIDPITCH].ki *(ctrl_angular_velocity.err.y - ctrl_angular_velocity.damp.y) *T;
%     ctrl_angular_velocity.err_i.z += ctrl_angular_velocity.PID[PIDYAW].ki 	*(ctrl_angular_velocity.err.z - ctrl_angular_velocity.damp.z) *T;
%    % /* 角速度误差积分分离 */
%     ctrl_angular_velocity.eliminate_I.x = Thr_Weight *CTRL_1_INT_LIMIT ;
%     ctrl_angular_velocity.eliminate_I.y = Thr_Weight *CTRL_1_INT_LIMIT ;
%     ctrl_angular_velocity.eliminate_I.z = Thr_Weight *CTRL_1_INT_LIMIT ;
%   %  /* 角速度误差积分限幅 */
%     ctrl_angular_velocity.err_i.x = LIMIT( ctrl_angular_velocity.err_i.x, -ctrl_angular_velocity.eliminate_I.x,ctrl_angular_velocity.eliminate_I.x );
%     ctrl_angular_velocity.err_i.y = LIMIT( ctrl_angular_velocity.err_i.y, -ctrl_angular_velocity.eliminate_I.y,ctrl_angular_velocity.eliminate_I.y );
%     ctrl_angular_velocity.err_i.z = LIMIT( ctrl_angular_velocity.err_i.z, -ctrl_angular_velocity.eliminate_I.z,ctrl_angular_velocity.eliminate_I.z );
%   %  /* 角速度PID输出 */
%     ctrl_angular_velocity.out.x = 3 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.x),0,1)*except_AS.x + ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDROLL].kp  *( ctrl_angular_velocity.err.x + ctrl_angular_velocity.err_d.x + ctrl_angular_velocity.err_i.x ) );
%     ctrl_angular_velocity.out.y = 3 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.y),0,1)*except_AS.y + ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDPITCH].kp *( ctrl_angular_velocity.err.y + ctrl_angular_velocity.err_d.y + ctrl_angular_velocity.err_i.y ) );
%     ctrl_angular_velocity.out.z = 3 *( ctrl_angular_velocity.FB *LIMIT((0.45f + 0.55f*ctrl_attitude.err_weight.z),0,1)*except_AS.z + ( 1 - ctrl_angular_velocity.FB ) *ctrl_angular_velocity.PID[PIDYAW].kp   *( ctrl_angular_velocity.err.z + ctrl_angular_velocity.err_d.z + ctrl_angular_velocity.err_i.z ) );
% 
%     %/* 角速度控制量 转换到 电机转速的输出量 */
%     All_Out(ctrl_angular_velocity.out.x,ctrl_angular_velocity.out.y,ctrl_angular_velocity.out.z);
%    % /* 记录角速度误差积分 */
%     ctrl_angular_velocity.err_old.x = ctrl_angular_velocity.err.x;
%     ctrl_angular_velocity.err_old.y = ctrl_angular_velocity.err.y;
%     ctrl_angular_velocity.err_old.z = ctrl_angular_velocity.err.z;
%  %   /* 记录角速度数据 */


    


    

    




  