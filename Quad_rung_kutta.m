function [x,y]=Quad_rung_kutta(ufunc,y0,h,a,b,p)%参数顺序依次为微分方程的函数名，初始值，步长，时间起点，时间终点，p是参数向量

n=floor((b-a)/h);  %n为步长，floor为取整函数
x(1)=a;            %时间起点
y(:,1)=y0;
for ii=1:n
    x(ii+1)=x(ii)+h;
    
    
    
    k1=ufunc(x(ii),y(:,ii),p);
    k2=ufunc(x(ii)+h/2,y(:,ii)+h*k1/2,p);
    k3=ufunc(x(ii)+h/2,y(:,ii)+h*k2/2,p);
    k4=ufunc(x(ii)+h/2,y(:,ii)+h*k3/2,p);
    y(:,ii+1)=y(:,ii)+h*(k1+2*k2+2*k3+k4)/6;
    
    
end