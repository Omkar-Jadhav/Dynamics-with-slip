function x = saturation(x,min,max)
%SATURATION Summary of this function goes here
%   Detailed explanation goes here

if(x<=min)
    x=min;
elseif(x>=max)
    x=max;
else
    x=x;
end

end

