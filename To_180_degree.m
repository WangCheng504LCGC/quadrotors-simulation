function y=To_180_degree(x)
if x>180
    y=x-360;
elseif x<-180;
    y=x+360;
else 
    y=x;
end