function [x,w]=Quad_AngularSpeedEquation(x,QuadPara,output,Dt,Df,w,h)%p是必要的参数
Ix=QuadPara.Ixx;
Iy=QuadPara.Iyy;
Iz=QuadPara.Izz;
l=QuadPara.l;
b=QuadPara.b;
d=QuadPara.d;
Ir=QuadPara.Ir;
m=QuadPara.m;
g=QuadPara.g;

u=output;
PWM(1)=-output(2)+output(1)-output(3)+output(4)+490;
PWM(2)=+output(2)+output(1)+output(3)+output(4)+490;
PWM(3)=+output(2)-output(1)-output(3)+output(4)+490;
PWM(4)=-output(2)-output(1)+output(3)+output(4)+490;

PWM(1)=LIMIT(PWM(1),0,1000); 
PWM(2)=LIMIT(PWM(2),0,1000); 
PWM(3)=LIMIT(PWM(3),0,1000); 
PWM(4)=LIMIT(PWM(4),0,1000); 

Tm=QuadPara.Tm;
wb=QuadPara.wb;
Cr=QuadPara.Cr;
poxi=zeros(4,1);
for i=1:4
poxi(i)=PWM(i)/1000;
w(i)=w(i)+(Cr*poxi(i)+wb-w(i))*h/Tm;
end
u(1)=b*l*(-w(4)*w(4)+w(2)*w(2)-w(3)*w(3)+w(1)*w(1))/sqrt(2);%俯仰方向
u(2)=b*l*(-w(4)*w(4)+w(2)*w(2)+w(3)*w(3)-w(1)*w(1))/sqrt(2);%横滚方向
u(3)=d*(w(2)*w(2)-w(1)*w(1)+w(4)*w(4)-w(3)*w(3));
u(4)=b*(w(2)*w(2)+w(1)*w(1)+w(4)*w(4)+w(3)*w(3));

if x(12)<=0 && u(4)<(m*g)
    u(4)=m*g;
    x(12)=0;
    x(9)=0;
end

wsum=0;%w(2)+w(4)-w(1)-w(3);
wx=x(1);
wy=x(2);
wz=x(3);
fai=x(4);
sita=x(5);
psai=x(6);
vx=x(7);
vy=x(8);
vz=x(9);
px=x(10);
py=x(11);
pz=x(12);

dwx=(Iy-Iz)*wz*wy+u(1)+Dt(1)+Ir*wy*wsum;
dwy=(Iz-Ix)*wx*wz+u(2)+Dt(2)-Ir*wx*wsum;
dwz=(Ix-Iy)*wx*wy+u(3)+Dt(3);

wx=wx+h*dwx/Ix;
wy=wy+h*dwy/Iy;
wz=wz+h*dwz/Iz;

% dfai=wx;
% dsita=wy;
% dpsai=wz;
dfai=wx+(wz*cos(fai)+wy*sin(fai))*tan(sita);
dsita=wy*cos(fai)-wz*sin(fai);
dpsai=1/cos(sita)*(wz*cos(fai)+wy*sin(fai));

fai=fai+h*dfai;   %Roll(北南)
sita=sita+h*dsita;%Pitch（西东）
psai=psai+h*dpsai;%Roll（逆时针）

ax=(cos(fai)*sin(sita)*cos(psai)+sin(fai)*sin(psai))*u(4)/m+Df(1);%正东方向
ay=(cos(fai)*sin(sita)*sin(psai)-sin(fai)*cos(psai))*u(4)/m+Df(2);%正北方向
az=(cos(sita)*cos(fai))*u(4)/m-g+Df(3);%

vx=vx+h*ax;
vy=vy+h*ay;
vz=vz+h*az;

px=px+h*vx;
py=py+h*vy;
pz=pz+h*vz;



x(1)=wx;
x(2)=wy;
x(3)=wz;
x(4)=fai;
x(5)=sita;
x(6)=psai;
x(7)=vx;
x(8)=vy;
x(9)=vz;
x(10)=px;
x(11)=py;
x(12)=pz;




