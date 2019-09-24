function y=Set_ADRC_Aspeed_Variable()
%PID²ÎÊý¸³Öµ
ADRC_Aspeed.h=0.002; 
ADRC_Aspeed.h0=10*ADRC_Aspeed.h; 

ADRC_Aspeed.b0= 0;
ADRC_Aspeed.beita01 = 0;
ADRC_Aspeed.beita02 = 0;
ADRC_Aspeed.beita03 = 0;

ADRC_Aspeed.r0 =  0;
ADRC_Aspeed.rz =  0;
ADRC_Aspeed.beita0z1 = 0;
ADRC_Aspeed.beita0z2 = 0;
ADRC_Aspeed.beita0z3 = 0;

ADRC_Aspeed.temp.x =  0;
ADRC_Aspeed.temp.y =  0;
ADRC_Aspeed.temp.z =  0;

ADRC_Aspeed.gyro_old.x =  0;
ADRC_Aspeed.gyro_old.y =  0;
ADRC_Aspeed.gyro_old.z =  0;

ADRC_Aspeed.v1.x =  0;
ADRC_Aspeed.v1.y =  0;
ADRC_Aspeed.v1.z =  0;

ADRC_Aspeed.err.x =  0;
ADRC_Aspeed.err.y =  0;
ADRC_Aspeed.err.z =  0;

ADRC_Aspeed.err_d.x =  0;
ADRC_Aspeed.err_d.y =  0;
ADRC_Aspeed.err_d.z =  0;

ADRC_Aspeed.err_i.x =  0;
ADRC_Aspeed.err_i.y =  0;
ADRC_Aspeed.err_i.z =  0;

ADRC_Aspeed.err_weight.x =0;
ADRC_Aspeed.err_weight.y =0;
ADRC_Aspeed.err_weight.z =0;

ADRC_Aspeed.v2.x = 0;
ADRC_Aspeed.v2.y = 0;
ADRC_Aspeed.v2.z = 0;

ADRC_Aspeed.z1.x = 0;
ADRC_Aspeed.z2.x = 0;
ADRC_Aspeed.z3.x = 0;

ADRC_Aspeed.z1.y = 0;
ADRC_Aspeed.z2.y = 0;
ADRC_Aspeed.z3.y = 0;

ADRC_Aspeed.z1.z  = 0;
ADRC_Aspeed.z2.z  = 0;
ADRC_Aspeed.z3.z  = 0;

ADRC_Aspeed.ui.x = 0;
ADRC_Aspeed.ui.y = 0;
ADRC_Aspeed.ui.z = 0;

ADRC_Aspeed.u0.x = 0;
ADRC_Aspeed.u1.x = 0;
ADRC_Aspeed.u2.x = 0;
ADRC_Aspeed.u3.x = 0;


ADRC_Aspeed.u0.y = 0;
ADRC_Aspeed.u1.y = 0;
ADRC_Aspeed.u2.y = 0;
ADRC_Aspeed.u3.y = 0;

ADRC_Aspeed.u0.z = 0;
ADRC_Aspeed.u1.z = 0;
ADRC_Aspeed.u2.z = 0;
ADRC_Aspeed.u3.z = 0;

ADRC_Aspeed.T0.x = 0;
ADRC_Aspeed.T1.x = 0;
ADRC_Aspeed.T2.x = 0;
ADRC_Aspeed.T3.x = 0;


ADRC_Aspeed.T0.y = 0;
ADRC_Aspeed.T1.y = 0;
ADRC_Aspeed.T2.y = 0;
ADRC_Aspeed.T3.y = 0;

ADRC_Aspeed.T0.z = 0;
ADRC_Aspeed.T1.z = 0;
ADRC_Aspeed.T2.z = 0;
ADRC_Aspeed.T3.z = 0;

ADRC_Aspeed.F0.x = 0;
ADRC_Aspeed.F1.x = 0;
ADRC_Aspeed.F2.x = 0;
ADRC_Aspeed.F3.x = 0;


ADRC_Aspeed.F0.y = 0;
ADRC_Aspeed.F1.y = 0;
ADRC_Aspeed.F2.y = 0;
ADRC_Aspeed.F3.y = 0;

ADRC_Aspeed.F0.z = 0;
ADRC_Aspeed.F1.z = 0;
ADRC_Aspeed.F2.z = 0;
ADRC_Aspeed.F3.z = 0;

ADRC_Aspeed.S24 =  0;
ADRC_Aspeed.S23 =  0;
ADRC_Aspeed.S14 =  0;
ADRC_Aspeed.S13 =  0;
ADRC_Aspeed.S41 =  0;
ADRC_Aspeed.S42 =  0;

y=ADRC_Aspeed;


