clc
clear all
L=1
p=1500
Axc=1
T=100
u0=.1

t=linspace(0,.007,200)
x=linspace(0,1,200)




for i=0:length(t)
    
    if i==0:
     u(i,i )=0
    
    for j=1:5
    
    A(j)=4*u0*L^2*(1-cos(pi*j));
    c=(T/p)^(1/2);
    

    u(i,i)=u(i,i) + A(j)  * sin(j*pi*x(i)/L) * cos(j*pi*c*t(i)/L);
    end
end


  

