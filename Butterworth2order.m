function Buffer=Butterworth2order(Buffer,data_input)%p是必要的参数
Buffer.Input_Butter2=data_input;%//最新数据
if Buffer.LPB_Cnt>=0

%                 /* Butterworth滤波 */
Buffer.Output_Butter2=Buffer.b0 * Buffer.Input_Butter2+Buffer.b1 * Buffer.Input_Butter1 +Buffer.b2 * Buffer.Input_Butter0-Buffer.a1 * Buffer.Output_Butter1-Buffer.a2 * Buffer.Output_Butter0;
else
    Buffer.Output_Butter2=Buffer.Input_Butter2;
    Buffer.LPB_Cnt=Buffer.LPB_Cnt+1;
end
%   /* x(n)序列后退*/
Buffer.Input_Butter0=Buffer.Input_Butter1;
Buffer.Input_Butter1=Buffer.Input_Butter2;
%   /* y(n) 序列后退*/
Buffer.Output_Butter0=Buffer.Output_Butter1;
Buffer.Output_Butter1=Buffer.Output_Butter2;


