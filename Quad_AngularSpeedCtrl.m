function [W,CTRL]=Quad_AngularSpeedCtrl(ExpectValue,RealValue,p)%最后输出为转速
w=zeros(4,1);
old_wx=p(1);
old_wy=p(2);
old_wz=p(3);
dt=p(4);
ctrl_angular_speed_errIX=p(5);
ctrl_angular_speed_errIY=p(6);
ctrl_angular_speed_errIZ=p(7);

ctrl_angular_speed_roll_Kp=p(8);
ctrl_angular_speed_pitch_Kp=p(9);
ctrl_angular_speed_yaw_Kp=p(10);

ctrl_angular_speed_roll_Kd=p(11);
ctrl_angular_speed_pitch_Kd=p(12);
ctrl_angular_speed_yaw_Kd=p(13);

ctrl_angular_speed_roll_Ki=p(14);
ctrl_angular_speed_pitch_Ki=p(15);
ctrl_angular_speed_yaw_Ki=p(16);

wx=RealValue(1);
wy=RealValue(2);
wz=RealValue(3);

%求取控制量K项
ctrl_angular_speed_errX=ctrl_angular_speed_roll_Kp*(ExpectValue(1)-wx);
ctrl_angular_speed_errY=ctrl_angular_speed_pitch_Kp*(ExpectValue(2)-wy);
ctrl_angular_speed_errZ=ctrl_angular_speed_yaw_Kp*(ExpectValue(3)-wz);

%求取控制量D项
ctrl_angular_speed_dampX=(wx-old_wx)/dt;
ctrl_angular_speed_dampY=(wy-old_wy)/dt;
ctrl_angular_speed_dampZ=(wy-old_wz)/dt;

ctrl_angular_speed_errDX=ctrl_angular_speed_roll_Kd*ctrl_angular_speed_dampX;
ctrl_angular_speed_errDY=ctrl_angular_speed_pitch_Kd*ctrl_angular_speed_dampY;
ctrl_angular_speed_errDZ=ctrl_angular_speed_yaw_Kd*ctrl_angular_speed_dampZ;
%积分项
ctrl_angular_speed_errIX=ctrl_angular_speed_errIX+ctrl_angular_speed_roll_Ki*ctrl_angular_speed_dampX;
ctrl_angular_speed_errIY=ctrl_angular_speed_errIY+ctrl_angular_speed_pitch_Ki*ctrl_angular_speed_dampY;
ctrl_angular_speed_errIZ=ctrl_angular_speed_errIZ+ctrl_angular_speed_yaw_Ki*ctrl_angular_speed_dampZ;
%积分限幅
ctrl_angular_speed_errIX=LIMIT(ctrl_angular_speed_errIX,-150,150);
ctrl_angular_speed_errIY=LIMIT(ctrl_angular_speed_errIY,-150,150);
ctrl_angular_speed_errIZ=LIMIT(ctrl_angular_speed_errIZ,-150,150);
%角速度PID输出
ctrl_angular_speed_outX=ctrl_angular_speed_errX+ctrl_angular_speed_errDX+ctrl_angular_speed_errIX;
ctrl_angular_speed_outY=ctrl_angular_speed_errY+ctrl_angular_speed_errDY+ctrl_angular_speed_errIY;
ctrl_angular_speed_outZ=ctrl_angular_speed_errZ+ctrl_angular_speed_errDZ+ctrl_angular_speed_errIZ;

dwx=(Iy-Iz)*wz*wy+Ir*wy*wsum+l*b*(w(4)^2-w(2)^2);
dwy=(Iz-Ix)*wx*wz-Ir*wx*wsum+l*b*(w(3)^2-w(1)^2);
dwz=(Ix-Iy)*wx*wy+d*l*(-w(1)^2+w(2)^2-w(3)^2+w(4)^2);

dy(1)=dwx;
dy(2)=dwy;
dy(3)=dwz;