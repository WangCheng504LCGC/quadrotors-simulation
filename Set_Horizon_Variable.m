function y=Set_Horizon_Variable()%p是必要的参数
%PID参数赋值
ctrl_Horizon.h=0.005; 
ctrl_Horizon.h0=10*ctrl_Horizon.h; 

%PID控制积分项
ctrl_Horizon.i.x= 0;
ctrl_Horizon.i.y= 0;

ctrl_Horizon.b0= 0;
ctrl_Horizon.beita01 = 0;
ctrl_Horizon.beita02 = 0;

ctrl_Horizon.r0 =  0;
ctrl_Horizon.Vmax =  0;

ctrl_Horizon.v1 =  0;
ctrl_Horizon.v2 = 0;

ctrl_Horizon.z1 = 0;
ctrl_Horizon.z2 = 0;
ctrl_Horizon.z3= 0;




ctrl_Horizon.u0 = 0;
ctrl_Horizon.u.x = 0;
ctrl_Horizon.last_u.x = 0;
ctrl_Horizon.u.y = 0;
ctrl_Horizon.last_u.y = 0;

y=ctrl_Horizon;


