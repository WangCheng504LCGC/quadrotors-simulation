function y=fal(e,arf,eta)
s=0.5*(sgn(e+eta)-sgn(e-eta));
b=abs(e);
y=e*s/(eta^(1-arf))+(b^arf)*sgn(e)*(1-s);
% if arf==0.5
%     a=sqrt(eta);
%     b=eta/a;   
%     c=sqrt(abs(e));
%     y=e/(b)*s+c*sgn(e)*(1-s);
% elseif arf==0.25;
%     a=sqrt(eta);
%     a=sqrt(a);
%     b=eta/a;   
%     c=sqrt(abs(e));
%     c=sqrt(c);
%     y=e/(b)*s+c*sgn(e)*(1-s);
% else 
%     a=sqrt(eta);
%     b=eta/a;   
%     c=sqrt(abs(e));
%     y=e/(b)*s+c*sgn(e)*(1-s);
end