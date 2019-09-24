function dy=Quad_AngleEquation(x,y,p)%p是必要的参数
dy=zeros(3,1);
wx=p(1);
wy=p(2);
wz=p(3);
psai=y(1);
fai=y(2);
sita=y(3);

dpsai=wx+(wz*cos(psai)+wy*sin(psai)*tan(sita));
dsita=wy*cos(psai)-wz*sin(psai);
dfai=(wz*cos(psai)+wy*sin(psai))/cos(sita);
dy(1)=dpsai;
dy(2)=dfai;
dy(3)=dsita;