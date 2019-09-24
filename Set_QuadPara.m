function y=Set_QuadPara()%p是必要的参数
QuadPara.m=1.076;%无人机质量-单位：kg
QuadPara.g=9.8;%重力加速度-单位：m/s^2
QuadPara.Ixx=1.253e-2;%绕x轴转动惯量：kg.m^2
QuadPara.Iyy=1.253e-2;%绕y轴转动惯量单位：kg.m^2
QuadPara.Izz=2.359e-2;%绕x轴转动惯量单位：kg.m^2
QuadPara.Ir=1.30e-4;%螺旋桨自身转动惯量单位：kg.m^2
QuadPara.b=1.026e-5;%螺旋桨升力系数转动惯量单位：N/(rad/s)^2
QuadPara.d=1.313e-7;%螺旋桨阻力系数转动惯量单位：N.m/(rad/s)^2
QuadPara.l=0.225;%质心到电机轴之间的距离：m
QuadPara.Tm=0.1;%电机响应时间常数：s
QuadPara.Cr=712.52;%电机响应时间常数：rad/s
QuadPara.wb=131.67;%电机响应时间常数：rad/s

QuadPara.PWM1=0;%电机1转速：rad/s
QuadPara.PWM2=0;%电机2转速：rad/s
QuadPara.PWM3=0;%电机3转速：rad/s
QuadPara.PWM4=0;%电机4转速：rad/s

QuadPara.w1=0;%电机1转速：rad/s
QuadPara.w2=200;%电机2转速：rad/s
QuadPara.w3=0;%电机3转速：rad/s
QuadPara.w4=200;%电机4转速：rad/s
QuadPara.wsum=0;

QuadPara.a1=(QuadPara.Iyy-QuadPara.Izz)/QuadPara.Ixx;
QuadPara.a2=(QuadPara.Izz-QuadPara.Ixx)/QuadPara.Iyy;
QuadPara.a3=(QuadPara.Ixx-QuadPara.Iyy)/QuadPara.Izz;
QuadPara.b1=1/QuadPara.Ixx;
QuadPara.b2=1/QuadPara.Iyy;
QuadPara.b3=1/QuadPara.Izz;

QuadPara.Dtx=0;%干扰力矩
QuadPara.Dty=0;%
QuadPara.Dtz=0;%
QuadPara.Dfx=0;%干扰力
QuadPara.Dfy=0;%干扰力
QuadPara.Dfz=0;%干扰力
y=QuadPara;