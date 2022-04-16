function [x,y] = myrunge_kutta(fun,x0,xt,y0,pointnum)
%UNTITLED it is a forward dynamics solve method ODE45
%   fun:函数f(x,y)
%   自变量的初值和终值：0,xt
%   自变量在[x0,xt]上的取值：pointnum
%   函数在x处的值：y0
%   x：所有取点的值
%   y：对应点上的函数值

y(1,:)=y0(:)' ;
h=(xt-x0)/(pointnum-1);
x=x0+[0:pointnum]'*h;
for k =1:pointnum
    f1=h * feval( fun,x(k) , y( k,: ) ) ;
    f1=f1(:)' ;
    f2=h * feval( fun,x(k) +h/2, y( k,: )+f1/2 ) ;
    f2=f2(:)' ;
    f3=h * feval( fun,x(k)+h/2 , y( k,: )+f2/2  ) ;
    f3=f3(:)' ;
    f4=h * feval( fun,x(k)+h , y( k,: )+f3 ) ;
    f4=f4(:)' ;
    
    y( k+1,: )=y( k,: ) + (f1 + 2 * ( f2+f3 ) + f4 ) / 6 ;
    
    
end

end

