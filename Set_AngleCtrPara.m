function y1=Set_AngleCtrPara()%p是必要的参数
%PID参数赋值
ctrl_attitude.PIDROLL.kp=0.5; ctrl_attitude.PIDPITCH.kp=0.5; ctrl_attitude.PIDYAW.kp=0.05;
ctrl_attitude.PIDROLL.ki=0.05;ctrl_attitude.PIDPITCH.ki=0.05;ctrl_attitude.PIDYAW.ki=0.01;
ctrl_attitude.PIDROLL.kd=0.3;ctrl_attitude.PIDPITCH.kd=0.3;ctrl_attitude.PIDYAW.kd=0.1;

ctrl_attitude.err.x=0;
ctrl_attitude.err.y=0;
ctrl_attitude.err.z=0;

% 计算角度误差权重 %
ctrl_attitude.err_weight.x = 0;
ctrl_attitude.err_weight.y = 0;
ctrl_attitude.err_weight.z = 0;
% 角度误差微分（跟随误差曲线变化）% 
ctrl_attitude.err_d.x = 0;
ctrl_attitude.err_d.y = 0;
ctrl_attitude.err_d.z = 0;
% 角度误差积分 %
ctrl_attitude.err_i.x =0;
ctrl_attitude.err_i.y =0;
ctrl_attitude.err_i.z =0;
%角度误差积分分离 %
ctrl_attitude.eliminate_I.x = 0;
ctrl_attitude.eliminate_I.y = 0;
ctrl_attitude.eliminate_I.z = 0;
%角度误差积分限幅 %
ctrl_attitude.err_i.x =0;
ctrl_attitude.err_i.y = 0;
ctrl_attitude.err_i.z = 0;
%对用于计算比例项输出的角度误差限幅%
ctrl_attitude.err.x = 0;
ctrl_attitude.err.y = 0;
ctrl_attitude.err.z = 0;
% 角度PID输出 %
ctrl_attitude.out.x = 0;
ctrl_attitude.out.y =0;
ctrl_attitude.out.z = ctrl_attitude.PIDYAW.kp   *( ctrl_attitude.err.z + ctrl_attitude.err_d.z + ctrl_attitude.err_i.z );
%记录历史数据%
ctrl_attitude.err_old.x = 0;
ctrl_attitude.err_old.y = 0;
ctrl_attitude.err_old.z = 0;


y1=ctrl_attitude;

