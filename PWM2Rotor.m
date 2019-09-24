function dy=Quad_AngularSpeedEquation(x,y,QuadPara)%p�Ǳ�Ҫ�Ĳ���
dy=zeros(4,1);
w(1)=QuadPara.w1;%���1ת�٣�rad/s
w(2)=QuadPara.w2;%���2ת�٣�rad/s
w(3)=QuadPara.w3;%���3ת�٣�rad/s
w(4)=QuadPara.w4;%���4ת�٣�rad/s
Tm=QuadPara.Tm;
Cr=QuadPara.Cr;
wb=QuadPara.wb;
w(1)=QuadPara.w1;%���1ת�٣�rad/s
w(2)=QuadPara.w2;%���2ת�٣�rad/s
w(3)=QuadPara.w3;%���3ת�٣�rad/s
w(4)=QuadPara.w4;%���4ת�٣�rad/s

dwi=(Iy-Iz)*wz*wy+Ir*wy*wsum+l*b*(w(4)^2-w(2)^2)+QuadPara.Dtx;
dwy=(Iz-Ix)*wx*wz-Ir*wx*wsum+l*b*(w(3)^2-w(1)^2)+QuadPara.Dtx;
dwz=(Ix-Iy)*wx*wy+d*l*(-w(1)^2+w(2)^2-w(3)^2+w(4)^2)+QuadPara.Dty;

dy(1)=dwx;
dy(2)=dwy;
dy(3)=dwz;
