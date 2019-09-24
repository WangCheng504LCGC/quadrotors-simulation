function dy=Quad_PostionEquation(x,y,p)
dy=zeros(3,1);
Vx=p(1);
Vy=p(2);
Vz=p(3);

dPx=Vx;
dPy=Vy;
dPz=Vz;

dy(1)=dPx;
dy(2)=dPy;
dy(3)=dPz;