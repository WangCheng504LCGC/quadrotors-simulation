function yout=HorizonCtrl(FlyOrbit,ctrl_Horizon,x,h,FlyMission)%
%% 姿态角和模型参数
deg1=x(4);g=9.8;
deg3=x(6);
Sin_Yaw=sin(deg3);Cos_Yaw=cos(deg3);Cos_Sita=cos(deg1);
%% 偏差量计算
evx=0;
h0=3*h;   
FlyOrbit.r0=20;
if FlyMission==0
  evy=0;
  FlyOrbit.v1.x=0;
  FlyOrbit.v2.x=0;
else
    fh=fhan(FlyOrbit.v1.x-5,FlyOrbit.v2.x,FlyOrbit.r0,h0);
    FlyOrbit.v1.x=FlyOrbit.v1.x+h*FlyOrbit.v2.x;
    FlyOrbit.v2.x=FlyOrbit.v2.x+h*fh;    
    FlyOrbit.v2.x=LIMIT_Min_Max(FlyOrbit.v2.x,-1,1);   
    %FlyOrbit.v2.x=0.5*(1- x(11));
    vtar=0.4*(2- x(11));
    vtar=LIMIT_Min_Max(vtar,-1,1);
    evy=vtar-x(8);
%   fh=fhan(FlyOrbit.v1.x-evy,FlyOrbit.v2.x,FlyOrbit.r0,h0);
%   FlyOrbit.v1.x=FlyOrbit.v1.x+h*FlyOrbit.v2.x;
%   FlyOrbit.v2.x=FlyOrbit.v2.x+h*fh;
end



%% PID控制
kp=0.1;ki=0.01;kd=0.1;
%积分项
ctrl_Horizon.i.x=ctrl_Horizon.i.x+ki*h*evx;
ctrl_Horizon.i.y=ctrl_Horizon.i.y+ki*h*evy;
ctrl_Horizon.i.x=LIMIT_Min_Max(ctrl_Horizon.i.x,-1.5,1.5); 
ctrl_Horizon.i.y=LIMIT_Min_Max(ctrl_Horizon.i.y,-1.5,1.5); 
%PID
FlyOrbit.v2.y=evy-FlyOrbit.v1.y;
ux=ctrl_Horizon.i.x+kp*evx+kd*evx+FlyOrbit.ax;
uy=ctrl_Horizon.i.y+kp*evy+kd*FlyOrbit.v2.y;
FlyOrbit.v1.y=evy;
ux=LIMIT_Min_Max(ux,-50,50); 
uy=LIMIT_Min_Max(uy,-50,50);

ctrl_Horizon.u.x =ux*Cos_Yaw+uy*Sin_Yaw; 
ctrl_Horizon.u.y =ux*Sin_Yaw-uy*Cos_Yaw; 

yout=ctrl_Horizon;


  