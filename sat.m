function y=sat(a,d)

if abs(a)<=d
    y=a/d;
else
    y=sgn(a);
end