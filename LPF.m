clc
clear
a=0;
step=0.002;
j=0;
for i=0:step:5
  j=j+1;
  a(j+1)=a(j)+0.5*3.14*step*(1000-a(j));
  t(j)=j*0.002;
end
plot(t,a(1:2501));