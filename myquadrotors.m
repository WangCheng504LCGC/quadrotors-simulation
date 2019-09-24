%% DOCUMENT TITLE：实验室四旋翼无人机仿真程序
% INTRODUCTORY TEXT
%%
format short;%显示4位小数
%format rat %显示分数
comment=zeros(50,1);
comment(1)=0; 
if comment(1)==0
clc
clear
end
comment=zeros(50,1);      
comment(1:7)=1;
comment(8)=1;  %仿真总循环
comment(9)=3; %角速度环控制曲线显示,FS1:1,2,3-分别表示显示这三条曲线
comment(11)=1; %高度环控制曲线显示
comment(12)=2; %1表示姿态环单独控制，2表示位置环先算出姿态环期望值，二者结合控制
%comment(12:12)=0; 
comment(15:16)=0; 

%% 1.模型参数设置
if comment(2)==1
QuadPara=Set_QuadPara();%设置无人机基本参数
end
%% 2.相关变量设置
if comment(3)==1
%状态量：三个机体角速度、三个欧拉角速度、三方向速度、三方向位置
x=zeros(12,1);
%%% 四个控制输入
u=zeros(4,1);
%%% 四个电机转速
w=zeros(4,1);
%%% 外力矩干扰
Dt=zeros(3,1);
%%% 外力干扰
Df=zeros(3,1);
end
%% 3.位置环相关变量设置
if comment(4)==1
PosE=[0 0 1];
VelE=zeros(3,1);  
AccE=[0 0 0];
FlyOrbit=Set_FlyOrbit_Variable();          %飞行轨迹变量设置
ADRC_Height=Set_ADRC_Height_Variable();    %高度自抗扰控制的结构体变量
ctrl_Horizon=Set_Horizon_Variable();       %高度自抗扰控制的结构体变量
FlyMission=0;  %飞行指令，0-直线；1-绕圆
end
%% 4.姿态环相关变量设置
if comment(5)==1
AttE=zeros(3,1);
AspE=zeros(3,1);  
GyrE=zeros(3,1);  
ctrl_attitude=Set_AngleCtrPara();          %姿态环的结构体变量
ctrl_angular_velocity=Set_AngleSpCtrPara();%角速度环的结构体变量
ADRC_Aspeed=Set_ADRC_Aspeed_Variable();    %高度自抗扰控制的结构体变量
ADRC_Angle=Set_ADRC_Angle_Variable();    %高度自抗扰控制的结构体变量
% BufferPara=Set_500HzButterPara(2);            %巴特沃斯参数
% Buffer=Set_ButterworthPara();            %巴特沃斯结构体变量
% Buffer.b0=BufferPara.b0;
% Buffer.b1=BufferPara.b1;
% Buffer.b2=BufferPara.b2;
% Buffer.a1=BufferPara.a1;
% Buffer.a2=BufferPara.a2;
end
%% 5.系统仿真时间变量
if comment(6)==1
timet=25;                  %仿真时间
dt=0.001;                 %最小步长
T0 = 0:dt:timet;          %最长仿真步数
T1=0:2*dt:timet;          %机体角速度环仿真步数
T2=0:5*dt:timet;          %欧拉角环仿真步数
T3=0:5*dt:timet;         %高度环仿真步数
T4=0:10*dt:timet;         %水平位置环仿真步数
flag=0;                   %控制环标志
end
%% 6.绘图相关变量
if comment(7)==1
Fig_gyro=zeros(3,length(T1)+1);
Fig_gyroE=zeros(3,length(T1)+1);
Fig_U=zeros(4,length(T2)+1);
Fig_att=zeros(3,length(T2)+1);
Fig_attE=zeros(3,length(T2)+1);
Fig_vel=zeros(3,length(T3)+1);
Fig_velE=zeros(3,length(T3)+1);
Fig_pos=zeros(3,length(T3)+1);
Fig_posE=zeros(3,length(T3)+1);
Fig_posU=zeros(3,length(T4)+1);
k1=1;%角速度环计数；
k2=1;%角度环计数；
k3=1;%位置速度环计数；
end
%% 7.仿真工作开始
if comment(8)==1
for i=0:length(T0)+1
    flag=flag+1;   
    if rem(flag,1)==0   %rem取余函数 ，系统运动方程，每毫秒执行一次        
        [x,w]=Quad_AngularSpeedEquation(x,QuadPara,u,Dt,Df,w,dt);%
    end 
    if i>2000   %rem取余函数 ，系统运动方程，每毫秒执行一次        
      Dt(2)=0;%
    end 
    if rem(flag,2)==0   %rem取余函数  ，机体角速度环，每2毫秒执行一次
       if comment(9)==1
        asp=30;
        if FS1==1
          ADRC_Angle.u0.x =asp;
        elseif FS1==2
          ADRC_Angle.u0.y =asp;   
        else
          ADRC_Angle.u0.z =asp;    
        end
        
       end           
        
        [ADRC_Aspeed]=AngSpeedCtrl(QuadPara,ADRC_Angle,ADRC_Aspeed,x,2*dt);%   
        u(1)=ADRC_Aspeed.u3.x;
        u(2)=ADRC_Aspeed.u3.y;
        u(3)=ADRC_Aspeed.u3.z;     
        if comment(9)==1
        Fig_gyro(:,k1)=x(1:3)*180/pi;
        Fig_gyroE(1,k1)=ADRC_Aspeed.v1.x;
        Fig_gyroE(2,k1)=ADRC_Aspeed.v1.y;
        Fig_gyroE(3,k1)=x(6)*180/pi;%/ADRC_Aspeed.v1.z;    
        end        
        k1=k1+1;
    end 
    if rem(flag,5)==0  %rem取余函数
       if comment(9)==2
       AttE(1)=0;
       AttE(2)=30;
       AttE(3)=0;
       FS1=2;
       end      
       if comment(9)==2||comment(9)==3
       %姿态角环，每5毫秒执行一次       
       [ADRC_Angle,ADRC_Aspeed]=AngCtrl(AttE,ADRC_Angle,ADRC_Aspeed,x,5*dt);%           

       Fig_att(:,k2)=x(4:6)*180/pi;
       Fig_attE(:,k2)=x(2);%AttE;      
       end
       %高度环,自抗扰控制，也是每5毫秒执行一次 
       if comment(11)==1
       ADRC_Height=HeightCtrl(PosE(3),ADRC_Height,QuadPara,x,5*dt);%
       u(4)=ADRC_Height.u;
           if comment(12)~=2
           Fig_pos(3,k2)=x(12);
           Fig_vel(3,k2)=x(9);
           Fig_posE(3,k2)=ADRC_Height.v1;
           Fig_velE(3,k2)=ADRC_Height.v2;   
           Fig_posU(3,k2)=ADRC_Height.u;   
           end
       end        
       Fig_U(:,k2)=ADRC_Aspeed.z2.y;%u;    
       k2=k2+1;
    end    
    if rem(flag,10)==0
       if comment(12)==2
       %水平位置航迹指令设计：0-悬停；1-直线，2-绕圆
       if i/1000>5
         FlyMission=1;  %
       else
         FlyMission=0;  %水平位置期望值为0
       end
      % FlyOrbit=TrajeGenerate(FlyOrbit,FlyMission,10*dt);
       %水平位置控制器        
       ctrl_Horizon=HorizonCtrl(FlyOrbit,ctrl_Horizon,x,10*dt,FlyMission);%
       AttE(1)=ctrl_Horizon.u.y*180/pi;
       AttE(2)=ctrl_Horizon.u.x*180/pi;        

       
       Fig_pos(1:2,k3)=x(10:11);
       Fig_vel(1:2,k3)=x(7:8);
       Fig_posE(1,k3)=FlyOrbit.v1.x;
       Fig_posE(2,k3)=0.1;%FlyOrbit.v1.y;
       Fig_velE(1,k3)=FlyOrbit.v2.x;
       Fig_velE(2,k3)=FlyOrbit.v2.x;%FlyOrbit.v2.y;
       Fig_posU(2,k3)=ctrl_Horizon.u.y ;
       end 
       k3=k3+1;
    end
end
end

if comment(9)==1 %%显示角速度环曲线
figure
plot(T1,Fig_gyroE(FS1,1:length(T1)),'Color',[1,0,0],'LineWidth',1.5)%红色
hold on
plot(T1,Fig_gyro(FS1,1:length(T1)),'Color',[0,0.75,0.75],'LineWidth',1.5)%青色
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(9)==2 %%显示角度环曲线
figure
plot(T2,Fig_attE(FS1,1:length(T2)),'Color',[1,0,0],'LineWidth',1.5)%红色
hold on
plot(T2,Fig_att(1,1:length(T2)),'Color',[0,0.75,0.75],'LineWidth',1.5)%青色
hold on
plot(T2,Fig_att(2,1:length(T2)),'Color',[0,0.5,0],'LineWidth',1.5)%深绿
hold on
plot(T2,Fig_att(3,1:length(T2)),'Color',[0,0,1],'LineWidth',1.5)%蓝色
hold on
plot(T2,Fig_U(FS1,1:length(T2)),'Color',[0,0,1],'LineWidth',1.5)%
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(11)==1&&comment(12)~=2%% %%显示高度环曲线
figure
plot(T3(1:length(T3)-2),Fig_posE(3,1:length(T3)-2),'Color',[1,0,0],'LineWidth',1.5)%红色
hold on
plot(T3(1:length(T3)-2),Fig_velE(3,1:length(T3)-2),'Color',[0,0.75,0.75],'LineWidth',1.5)%青色
hold on
plot(T3(1:length(T3)-2),Fig_pos(3,1:length(T3)-2),'Color',[0,0.5,0],'LineWidth',1.5)%深绿
hold on
plot(T3(1:length(T3)-2),Fig_vel(3,1:length(T3)-2),'Color',[0,0.5,1],'LineWidth',1.5)%蓝色
hold on
plot(T2,Fig_U(4,1:length(T2)),'Color',[0,0.5,1],'LineWidth',1.5)%
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(12)==2 %%显示水平环曲线
figure
plot(T4(1:length(T4)-2),Fig_posE(2,1:length(T4)-2),'Color',[1,0,0],'LineWidth',1.5)%红色
hold on
plot(T4(1:length(T4)-2),Fig_velE(2,1:length(T4)-2),'Color',[0,0.75,0.75],'LineWidth',1.5)%青色
hold on
plot(T4(1:length(T4)-2),Fig_pos(2,1:length(T4)-2),'Color',[0,0.5,0],'LineWidth',1.5)%深绿
hold on
plot(T4(1:length(T4)-2),Fig_vel(2,1:length(T4)-2),'Color',[0,0.5,1],'LineWidth',1.5)%蓝色
hold on
plot(T4,Fig_posU(2,1:length(T4)),'Color',[0,0.5,1],'LineWidth',1.5)%
set(gca,'LineWidth',1.5)
box on
grid on
end
% plot3(fig_Vx1,fig_Vy1,fig_Vz1);
% axis auto%([-0.5 7  -0.5 7 0 1.5]);
% grid on
% h2=0.005;
% plot(0:h2:h2*(length(fig_Vx1)-1),fig_Vx1,'k');
% hold on
% plot(0:h2:h2*(length(fig_Vx1)-1),fig_Vx1,'g');
% hold on 
% plot(0:h2:h2*(length(fig_Vx2)-1),fig_Vx2,'b');
% hold on
% plot(0:h2:h2*(length(fig_Rx)-1),fig_Rx,'r');
% hold on
% plot(0:h2:h2*(length(fig_Rvx)-1),fig_Rvx,':r');
