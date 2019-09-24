clc
clear


h=0.002;
T=10;
Tc=0.5;
y(1)=0;
u(1)=0;
u(2)=0;
y(2)=0;
k=1;
num=1.0e-07 *[  0   7.8005  7.6972];den =[ 1.0000   -1.9604    0.9608];
for i=0:h:T    
    k=k+1;
    u(k)=500;
    u(k+1)=500;
    y(k+1)=-den(2)*y(k)-den(3)*y(k-1)+num(1)*u(k+1)+num(2)*u(k)+num(3)*u(k-1);
end
plot(0:h:h*(length(u)-1),y,'r');
% hold on
% plot(0:h:h*(length(y)-1),u,'b');
    