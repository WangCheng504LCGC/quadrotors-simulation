function y1=Set_AngleCtrPara()%p�Ǳ�Ҫ�Ĳ���
%PID������ֵ
ctrl_attitude.PIDROLL.kp=0.5; ctrl_attitude.PIDPITCH.kp=0.5; ctrl_attitude.PIDYAW.kp=0.05;
ctrl_attitude.PIDROLL.ki=0.05;ctrl_attitude.PIDPITCH.ki=0.05;ctrl_attitude.PIDYAW.ki=0.01;
ctrl_attitude.PIDROLL.kd=0.3;ctrl_attitude.PIDPITCH.kd=0.3;ctrl_attitude.PIDYAW.kd=0.1;

ctrl_attitude.err.x=0;
ctrl_attitude.err.y=0;
ctrl_attitude.err.z=0;

% ����Ƕ����Ȩ�� %
ctrl_attitude.err_weight.x = 0;
ctrl_attitude.err_weight.y = 0;
ctrl_attitude.err_weight.z = 0;
% �Ƕ����΢�֣�����������߱仯��% 
ctrl_attitude.err_d.x = 0;
ctrl_attitude.err_d.y = 0;
ctrl_attitude.err_d.z = 0;
% �Ƕ������� %
ctrl_attitude.err_i.x =0;
ctrl_attitude.err_i.y =0;
ctrl_attitude.err_i.z =0;
%�Ƕ������ַ��� %
ctrl_attitude.eliminate_I.x = 0;
ctrl_attitude.eliminate_I.y = 0;
ctrl_attitude.eliminate_I.z = 0;
%�Ƕ��������޷� %
ctrl_attitude.err_i.x =0;
ctrl_attitude.err_i.y = 0;
ctrl_attitude.err_i.z = 0;
%�����ڼ������������ĽǶ�����޷�%
ctrl_attitude.err.x = 0;
ctrl_attitude.err.y = 0;
ctrl_attitude.err.z = 0;
% �Ƕ�PID��� %
ctrl_attitude.out.x = 0;
ctrl_attitude.out.y =0;
ctrl_attitude.out.z = ctrl_attitude.PIDYAW.kp   *( ctrl_attitude.err.z + ctrl_attitude.err_d.z + ctrl_attitude.err_i.z );
%��¼��ʷ����%
ctrl_attitude.err_old.x = 0;
ctrl_attitude.err_old.y = 0;
ctrl_attitude.err_old.z = 0;


y1=ctrl_attitude;

