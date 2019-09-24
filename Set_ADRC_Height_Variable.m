function y=Set_ADRC_Height_Variable()%p是必要的参数
%PID参数赋值
ADRC_Height.h=0.005; 
ADRC_Height.h0=10*ADRC_Height.h; 

ADRC_Height.b0= 0;
ADRC_Height.beita01 = 0;
ADRC_Height.beita02 = 0;

ADRC_Height.r0 =  0;
ADRC_Height.Vmax =  0;

ADRC_Height.v1 =  0;
ADRC_Height.v2 = 0;

ADRC_Height.z1 = 0;
ADRC_Height.z2 = 0;
ADRC_Height.z3= 0;

ADRC_Height.u0 = 0;
ADRC_Height.u = 0;
ADRC_Height.last_u = 0;

y=ADRC_Height;


