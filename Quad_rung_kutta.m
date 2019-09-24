function [x,y]=Quad_rung_kutta(ufunc,y0,h,a,b,p)%����˳������Ϊ΢�ַ��̵ĺ���������ʼֵ��������ʱ����㣬ʱ���յ㣬p�ǲ�������

n=floor((b-a)/h);  %nΪ������floorΪȡ������
x(1)=a;            %ʱ�����
y(:,1)=y0;
for ii=1:n
    x(ii+1)=x(ii)+h;
    
    
    
    k1=ufunc(x(ii),y(:,ii),p);
    k2=ufunc(x(ii)+h/2,y(:,ii)+h*k1/2,p);
    k3=ufunc(x(ii)+h/2,y(:,ii)+h*k2/2,p);
    k4=ufunc(x(ii)+h/2,y(:,ii)+h*k3/2,p);
    y(:,ii+1)=y(:,ii)+h*(k1+2*k2+2*k3+k4)/6;
    
    
end