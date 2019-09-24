%% DOCUMENT TITLE��ʵ�������������˻��������
% INTRODUCTORY TEXT
%%
format short;%��ʾ4λС��
%format rat %��ʾ����
comment=zeros(50,1);
comment(1)=0; 
if comment(1)==0
clc
clear
end
comment=zeros(50,1);      
comment(1:7)=1;
comment(8)=1;  %������ѭ��
comment(9)=3; %���ٶȻ�����������ʾ,FS1:1,2,3-�ֱ��ʾ��ʾ����������
comment(11)=1; %�߶Ȼ�����������ʾ
comment(12)=2; %1��ʾ��̬���������ƣ�2��ʾλ�û��������̬������ֵ�����߽�Ͽ���
%comment(12:12)=0; 
comment(15:16)=0; 

%% 1.ģ�Ͳ�������
if comment(2)==1
QuadPara=Set_QuadPara();%�������˻���������
end
%% 2.��ر�������
if comment(3)==1
%״̬��������������ٶȡ�����ŷ�����ٶȡ��������ٶȡ�������λ��
x=zeros(12,1);
%%% �ĸ���������
u=zeros(4,1);
%%% �ĸ����ת��
w=zeros(4,1);
%%% �����ظ���
Dt=zeros(3,1);
%%% ��������
Df=zeros(3,1);
end
%% 3.λ�û���ر�������
if comment(4)==1
PosE=[0 0 1];
VelE=zeros(3,1);  
AccE=[0 0 0];
FlyOrbit=Set_FlyOrbit_Variable();          %���й켣��������
ADRC_Height=Set_ADRC_Height_Variable();    %�߶��Կ��ſ��ƵĽṹ�����
ctrl_Horizon=Set_Horizon_Variable();       %�߶��Կ��ſ��ƵĽṹ�����
FlyMission=0;  %����ָ�0-ֱ�ߣ�1-��Բ
end
%% 4.��̬����ر�������
if comment(5)==1
AttE=zeros(3,1);
AspE=zeros(3,1);  
GyrE=zeros(3,1);  
ctrl_attitude=Set_AngleCtrPara();          %��̬���Ľṹ�����
ctrl_angular_velocity=Set_AngleSpCtrPara();%���ٶȻ��Ľṹ�����
ADRC_Aspeed=Set_ADRC_Aspeed_Variable();    %�߶��Կ��ſ��ƵĽṹ�����
ADRC_Angle=Set_ADRC_Angle_Variable();    %�߶��Կ��ſ��ƵĽṹ�����
% BufferPara=Set_500HzButterPara(2);            %������˹����
% Buffer=Set_ButterworthPara();            %������˹�ṹ�����
% Buffer.b0=BufferPara.b0;
% Buffer.b1=BufferPara.b1;
% Buffer.b2=BufferPara.b2;
% Buffer.a1=BufferPara.a1;
% Buffer.a2=BufferPara.a2;
end
%% 5.ϵͳ����ʱ�����
if comment(6)==1
timet=25;                  %����ʱ��
dt=0.001;                 %��С����
T0 = 0:dt:timet;          %����沽��
T1=0:2*dt:timet;          %������ٶȻ����沽��
T2=0:5*dt:timet;          %ŷ���ǻ����沽��
T3=0:5*dt:timet;         %�߶Ȼ����沽��
T4=0:10*dt:timet;         %ˮƽλ�û����沽��
flag=0;                   %���ƻ���־
end
%% 6.��ͼ��ر���
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
k1=1;%���ٶȻ�������
k2=1;%�ǶȻ�������
k3=1;%λ���ٶȻ�������
end
%% 7.���湤����ʼ
if comment(8)==1
for i=0:length(T0)+1
    flag=flag+1;   
    if rem(flag,1)==0   %remȡ�ຯ�� ��ϵͳ�˶����̣�ÿ����ִ��һ��        
        [x,w]=Quad_AngularSpeedEquation(x,QuadPara,u,Dt,Df,w,dt);%
    end 
    if i>2000   %remȡ�ຯ�� ��ϵͳ�˶����̣�ÿ����ִ��һ��        
      Dt(2)=0;%
    end 
    if rem(flag,2)==0   %remȡ�ຯ��  ��������ٶȻ���ÿ2����ִ��һ��
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
    if rem(flag,5)==0  %remȡ�ຯ��
       if comment(9)==2
       AttE(1)=0;
       AttE(2)=30;
       AttE(3)=0;
       FS1=2;
       end      
       if comment(9)==2||comment(9)==3
       %��̬�ǻ���ÿ5����ִ��һ��       
       [ADRC_Angle,ADRC_Aspeed]=AngCtrl(AttE,ADRC_Angle,ADRC_Aspeed,x,5*dt);%           

       Fig_att(:,k2)=x(4:6)*180/pi;
       Fig_attE(:,k2)=x(2);%AttE;      
       end
       %�߶Ȼ�,�Կ��ſ��ƣ�Ҳ��ÿ5����ִ��һ�� 
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
       %ˮƽλ�ú���ָ����ƣ�0-��ͣ��1-ֱ�ߣ�2-��Բ
       if i/1000>5
         FlyMission=1;  %
       else
         FlyMission=0;  %ˮƽλ������ֵΪ0
       end
      % FlyOrbit=TrajeGenerate(FlyOrbit,FlyMission,10*dt);
       %ˮƽλ�ÿ�����        
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

if comment(9)==1 %%��ʾ���ٶȻ�����
figure
plot(T1,Fig_gyroE(FS1,1:length(T1)),'Color',[1,0,0],'LineWidth',1.5)%��ɫ
hold on
plot(T1,Fig_gyro(FS1,1:length(T1)),'Color',[0,0.75,0.75],'LineWidth',1.5)%��ɫ
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(9)==2 %%��ʾ�ǶȻ�����
figure
plot(T2,Fig_attE(FS1,1:length(T2)),'Color',[1,0,0],'LineWidth',1.5)%��ɫ
hold on
plot(T2,Fig_att(1,1:length(T2)),'Color',[0,0.75,0.75],'LineWidth',1.5)%��ɫ
hold on
plot(T2,Fig_att(2,1:length(T2)),'Color',[0,0.5,0],'LineWidth',1.5)%����
hold on
plot(T2,Fig_att(3,1:length(T2)),'Color',[0,0,1],'LineWidth',1.5)%��ɫ
hold on
plot(T2,Fig_U(FS1,1:length(T2)),'Color',[0,0,1],'LineWidth',1.5)%
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(11)==1&&comment(12)~=2%% %%��ʾ�߶Ȼ�����
figure
plot(T3(1:length(T3)-2),Fig_posE(3,1:length(T3)-2),'Color',[1,0,0],'LineWidth',1.5)%��ɫ
hold on
plot(T3(1:length(T3)-2),Fig_velE(3,1:length(T3)-2),'Color',[0,0.75,0.75],'LineWidth',1.5)%��ɫ
hold on
plot(T3(1:length(T3)-2),Fig_pos(3,1:length(T3)-2),'Color',[0,0.5,0],'LineWidth',1.5)%����
hold on
plot(T3(1:length(T3)-2),Fig_vel(3,1:length(T3)-2),'Color',[0,0.5,1],'LineWidth',1.5)%��ɫ
hold on
plot(T2,Fig_U(4,1:length(T2)),'Color',[0,0.5,1],'LineWidth',1.5)%
set(gca,'LineWidth',1.5)
box on
grid on
end
if comment(12)==2 %%��ʾˮƽ������
figure
plot(T4(1:length(T4)-2),Fig_posE(2,1:length(T4)-2),'Color',[1,0,0],'LineWidth',1.5)%��ɫ
hold on
plot(T4(1:length(T4)-2),Fig_velE(2,1:length(T4)-2),'Color',[0,0.75,0.75],'LineWidth',1.5)%��ɫ
hold on
plot(T4(1:length(T4)-2),Fig_pos(2,1:length(T4)-2),'Color',[0,0.5,0],'LineWidth',1.5)%����
hold on
plot(T4(1:length(T4)-2),Fig_vel(2,1:length(T4)-2),'Color',[0,0.5,1],'LineWidth',1.5)%��ɫ
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
