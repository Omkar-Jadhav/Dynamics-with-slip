function f = friction(mew,s,N)
%FRICTION_CONSTANT Summary of this function goes here
%   Detailed explanation goes here

if(s<0.15 && s>-0.15)
   f=s*mew/0.15*N;
elseif(s>= 0.15 && s<=1 )
  f=-(s+0.15)*0.34*mew/0.85+mew;
  f=f*N;
elseif(s<=-0.15&& s>=-1)
    f= -(s-0.15)*0.34*mew/0.85-mew;
    f=f*N;
end

end

