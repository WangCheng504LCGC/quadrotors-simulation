function out=LIMIT(x,min,max)

if x<min
    out=min;
elseif x>max
    out=max;
else
    out=x;
end