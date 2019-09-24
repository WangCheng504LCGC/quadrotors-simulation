function y=Set_QuadPara()%p�Ǳ�Ҫ�Ĳ���
QuadPara.m=1.076;%���˻�����-��λ��kg
QuadPara.g=9.8;%�������ٶ�-��λ��m/s^2
QuadPara.Ixx=1.253e-2;%��x��ת��������kg.m^2
QuadPara.Iyy=1.253e-2;%��y��ת��������λ��kg.m^2
QuadPara.Izz=2.359e-2;%��x��ת��������λ��kg.m^2
QuadPara.Ir=1.30e-4;%����������ת��������λ��kg.m^2
QuadPara.b=1.026e-5;%����������ϵ��ת��������λ��N/(rad/s)^2
QuadPara.d=1.313e-7;%����������ϵ��ת��������λ��N.m/(rad/s)^2
QuadPara.l=0.225;%���ĵ������֮��ľ��룺m
QuadPara.Tm=0.1;%�����Ӧʱ�䳣����s
QuadPara.Cr=712.52;%�����Ӧʱ�䳣����rad/s
QuadPara.wb=131.67;%�����Ӧʱ�䳣����rad/s

QuadPara.PWM1=0;%���1ת�٣�rad/s
QuadPara.PWM2=0;%���2ת�٣�rad/s
QuadPara.PWM3=0;%���3ת�٣�rad/s
QuadPara.PWM4=0;%���4ת�٣�rad/s

QuadPara.w1=0;%���1ת�٣�rad/s
QuadPara.w2=200;%���2ת�٣�rad/s
QuadPara.w3=0;%���3ת�٣�rad/s
QuadPara.w4=200;%���4ת�٣�rad/s
QuadPara.wsum=0;

QuadPara.a1=(QuadPara.Iyy-QuadPara.Izz)/QuadPara.Ixx;
QuadPara.a2=(QuadPara.Izz-QuadPara.Ixx)/QuadPara.Iyy;
QuadPara.a3=(QuadPara.Ixx-QuadPara.Iyy)/QuadPara.Izz;
QuadPara.b1=1/QuadPara.Ixx;
QuadPara.b2=1/QuadPara.Iyy;
QuadPara.b3=1/QuadPara.Izz;

QuadPara.Dtx=0;%��������
QuadPara.Dty=0;%
QuadPara.Dtz=0;%
QuadPara.Dfx=0;%������
QuadPara.Dfy=0;%������
QuadPara.Dfz=0;%������
y=QuadPara;