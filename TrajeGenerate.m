function yout=TrajeGenerate(FlyOrbit,FlyMission,h)%
%% 直线运动x,y轨迹生成
if FlyMission==1
FlyOrbit.EndX=5;
h0=10*h;    
fh=fhan(FlyOrbit.v1.x-FlyOrbit.EndX,FlyOrbit.v2.x,FlyOrbit.r0,h0);
FlyOrbit.v1.x=FlyOrbit.v1.x+h*FlyOrbit.v2.x;
FlyOrbit.v2.x=FlyOrbit.v2.x+h*fh;
FlyOrbit.v2.x=LIMIT_Min_Max(FlyOrbit.v2.x,-FlyOrbit.Vmax,FlyOrbit.Vmax);  
FlyOrbit.ax=fh;
fh=fhan(FlyOrbit.v1.y-FlyOrbit.EndY,FlyOrbit.v2.y,FlyOrbit.r0,h0);
FlyOrbit.v1.y=FlyOrbit.v1.y+h*FlyOrbit.v2.y;
FlyOrbit.v2.y=FlyOrbit.v2.y+h*fh;
FlyOrbit.v2.y=LIMIT_Min_Max(FlyOrbit.v2.y,-FlyOrbit.Vmax,FlyOrbit.Vmax); 
FlyOrbit.ay=fh;
%% 绕圆运动x,y轨迹生成
elseif FlyMission==2   
L=2*pi*FlyOrbit.R;
fh=fhan(FlyOrbit.v1.c-L,FlyOrbit.v2.c,FlyOrbit.r0,h0);
FlyOrbit.v1.c=FlyOrbit.v1.c+h*FlyOrbit.v2.c;
FlyOrbit.v2.c=FlyOrbit.v2.c+h*fh;
FlyOrbit.v2.c=LIMIT_Min_Max(FlyOrbit.v2.c,-FlyOrbit.Vmax,FlyOrbit.Vmax);  

sita=FlyOrbit.v1.c/FlyOrbit.R+FlyOrbit.sita_org;

FlyOrbit.v1.x=FlyOrbit.x0+FlyOrbit.R*cos(sita);
FlyOrbit.v2.x=-dir*FlyOrbit.v2.c*sin(sita);
FlyOrbit.ax=-fh*sin(sita)-(FlyOrbit.v2.c*FlyOrbit.v2.c)/R*cos(sita);

FlyOrbit.v1.y=FlyOrbit.y0+FlyOrbit.R*sin(sita);
FlyOrbit.v2.y=FlyOrbit.v2.c*cos(sita);
FlyOrbit.ay=fh*cos(sita)-(FlyOrbit.v2.c*FlyOrbit.v2.c)/R*sin(sita);
else
FlyOrbit.v1.x=0;
FlyOrbit.v2.x=0; 
FlyOrbit.EndX=0;
FlyOrbit.v1.y=0;
FlyOrbit.v2.y=0;
FlyOrbit.EndY=0;
FlyOrbit.ax=0;
FlyOrbit.ay=0;    
end

yout=FlyOrbit;



  