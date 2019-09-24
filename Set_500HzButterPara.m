function y=Set_500HzButterPara(x)
if x==51 %   //500hz---51hz
ButterPara.a0=1;
ButterPara.a1=-1.153469;
ButterPara.a2=0.417460;
ButterPara.b0=0.065998;
ButterPara.b1= 0.131996;
ButterPara.b2=0.065998;
elseif x==30%   //500hz---30hz
ButterPara.a0=1;
ButterPara.a1= -0.8239;
ButterPara.a2=0.2942;
ButterPara.b0=0.1176;
ButterPara.b1= 0.2352;
ButterPara.b2=0.1176;
elseif x==10%   //500hz---30hz
ButterPara.a0=1;
ButterPara.a1=  -1.8229;
ButterPara.a2=  0.83738;
ButterPara.b0=0.0036126;
ButterPara.b1=0.0072251;
ButterPara.b2=0.0036126;
elseif x==2%   //500hz---2hz
ButterPara.a0=1;
ButterPara.a1=   -1.9645 ;
ButterPara.a2=  0.96508;
ButterPara.b0=0.00015513;
ButterPara.b1=0.00031026;
ButterPara.b2=0.00015513;
else%   //default:500hz---30hz
ButterPara.a0=1;
ButterPara.a1=-1.153469;
ButterPara.a2=0.417460;
ButterPara.b0=0.065998;
ButterPara.b1= 0.131996;
ButterPara.b2=0.065998;    
end
y=ButterPara;