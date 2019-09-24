function fn=fhan(x1,x2,r,h)%¼ûÊé107Ò³

d=h*r;
d0=h*d;
y=x1+h*x2;
a0=sqrt(d^2+8*r*abs(y));

if abs(y)<=d0
    a=x2+y/h;
else
    a=x2+0.5*(a0-d)*sgn(y);
end
fn=-r*sat(a,d);


