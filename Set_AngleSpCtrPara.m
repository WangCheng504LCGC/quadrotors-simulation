function y=Set_AngleSpCtrPara()%p是必要的参数
%PID参数赋值
ctrl_angular_velocity.PIDROLL.kp=0.8; ctrl_angular_velocity.PIDPITCH.kp=0.8; ctrl_angular_velocity.PIDYAW.kp=1.2;
ctrl_angular_velocity.PIDROLL.ki=0.1;ctrl_angular_velocity.PIDPITCH.ki=0.1;ctrl_angular_velocity.PIDYAW.ki=1;
ctrl_angular_velocity.PIDROLL.kd=2;ctrl_angular_velocity.PIDPITCH.kd=2;ctrl_angular_velocity.PIDYAW.kd=1;
ctrl_angular_velocity.FB=0.2;

ctrl_angular_velocity.es.x = 0;
ctrl_angular_velocity.es.y = 0;
ctrl_angular_velocity.es.z = 0;

ctrl_angular_velocity.damp.x = 0;
ctrl_angular_velocity.damp.y = 0;
ctrl_angular_velocity.damp.z = 0;

ctrl_angular_velocity.err.x =  0;
ctrl_angular_velocity.err.y =  0;
ctrl_angular_velocity.err.z =  0;

ctrl_angular_velocity.err_weight.x = 0;
ctrl_angular_velocity.err_weight.y = 0;
ctrl_angular_velocity.err_weight.z = 0;

ctrl_angular_velocity.err_d.x = 0;
ctrl_angular_velocity.err_d.y = 0;
ctrl_angular_velocity.err_d.z = 0;

ctrl_angular_velocity.err_i.x = 0;
ctrl_angular_velocity.err_i.y = 0;
ctrl_angular_velocity.err_i.z = 0;

ctrl_angular_velocity.eliminate_I.x =150 ;
ctrl_angular_velocity.eliminate_I.y =150 ;
ctrl_angular_velocity.eliminate_I.z =150 ;

ctrl_angular_velocity.out.x =0 ;
ctrl_angular_velocity.out.y =0 ;
ctrl_angular_velocity.out.z =0;

ctrl_angular_velocity.g_old.A_X = 0;
ctrl_angular_velocity.g_old.A_Y = 0 ;
ctrl_angular_velocity.g_old.A_Z = 0;
y=ctrl_angular_velocity;


