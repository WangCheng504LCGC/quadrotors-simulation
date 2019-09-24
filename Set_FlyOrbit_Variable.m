function y=Set_FlyOrbit_Variable()%p是必要的参数

FlyOrbit.StartX =  0;%轨迹起点位置X
FlyOrbit.StartY =  0;%轨迹起点位置Y
FlyOrbit.r0 =  2;%最大加速度
FlyOrbit.Vmax =  0;%最大速度
%直线轨迹变量
FlyOrbit.EndX = 0;%轨迹终点位置X
FlyOrbit.EndY = 0;%轨迹终点位置Y
FlyOrbit.v1.x = FlyOrbit.StartX;%轨迹实时位置
FlyOrbit.v1.y = FlyOrbit.StartY;%轨迹实时位置
FlyOrbit.v2.x = 0;%轨迹实时速度
FlyOrbit.v2.y = 0;%轨迹实时位置
%绕圆轨迹变量
FlyOrbit.R =  2;%绕圆半径
FlyOrbit.w =  0;%绕圆角速度

FlyOrbit.x0 =  0;%圆心位置x
FlyOrbit.y0 = 0; %圆心位置y

FlyOrbit.v1.c = 0;%轨迹实时位置
FlyOrbit.v2.c =0;%轨迹实时位置
FlyOrbit.v2.sita_org=0;%初始角度
FlyOrbit.ax =0;%加速度x
FlyOrbit.ay =0;%y方向加速度
y=FlyOrbit;


