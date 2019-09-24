function y=LIMIT_Min_Max(value,min,max)

if value>=max
    y=max;
else
    if value<min
       y=min;
    else
       y=value;
    end       
end