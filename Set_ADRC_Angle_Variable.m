function y=Set_ADRC_Angle_Variable()
%PID²ÎÊý¸³Öµ
ADRC_Angle.h=0.002; 
ADRC_Angle.h0=10*ADRC_Angle.h; 


ADRC_Angle.r0 =  0;
ADRC_Angle.rz =  0;

ADRC_Angle.v1.x =  0;
ADRC_Angle.v1.y =  0;
ADRC_Angle.v1.z =  0;

ADRC_Angle.v2.x = 0;
ADRC_Angle.v2.y = 0;

ADRC_Angle.v2.z = 0;



ADRC_Angle.u0.x = 0;
ADRC_Angle.u1.x = 0;
ADRC_Angle.u2.x = 0;
ADRC_Angle.u3.x = 0;


ADRC_Angle.u0.y = 0;
ADRC_Angle.u1.y = 0;
ADRC_Angle.u2.y = 0;
ADRC_Angle.u3.y = 0;

ADRC_Angle.u0.z = 0;
ADRC_Angle.u1.z = 0;
ADRC_Angle.u2.z = 0;
ADRC_Angle.u3.z = 0;

ADRC_Angle.S23 =  0;
ADRC_Angle.S22 =  0;
ADRC_Angle.S12 =  0;
ADRC_Angle.S13 =  0;
ADRC_Angle.S41 =  0;
ADRC_Angle.S42 =  0;

y=ADRC_Angle;


