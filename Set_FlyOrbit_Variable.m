function y=Set_FlyOrbit_Variable()%p�Ǳ�Ҫ�Ĳ���

FlyOrbit.StartX =  0;%�켣���λ��X
FlyOrbit.StartY =  0;%�켣���λ��Y
FlyOrbit.r0 =  2;%�����ٶ�
FlyOrbit.Vmax =  0;%����ٶ�
%ֱ�߹켣����
FlyOrbit.EndX = 0;%�켣�յ�λ��X
FlyOrbit.EndY = 0;%�켣�յ�λ��Y
FlyOrbit.v1.x = FlyOrbit.StartX;%�켣ʵʱλ��
FlyOrbit.v1.y = FlyOrbit.StartY;%�켣ʵʱλ��
FlyOrbit.v2.x = 0;%�켣ʵʱ�ٶ�
FlyOrbit.v2.y = 0;%�켣ʵʱλ��
%��Բ�켣����
FlyOrbit.R =  2;%��Բ�뾶
FlyOrbit.w =  0;%��Բ���ٶ�

FlyOrbit.x0 =  0;%Բ��λ��x
FlyOrbit.y0 = 0; %Բ��λ��y

FlyOrbit.v1.c = 0;%�켣ʵʱλ��
FlyOrbit.v2.c =0;%�켣ʵʱλ��
FlyOrbit.v2.sita_org=0;%��ʼ�Ƕ�
FlyOrbit.ax =0;%���ٶ�x
FlyOrbit.ay =0;%y������ٶ�
y=FlyOrbit;


