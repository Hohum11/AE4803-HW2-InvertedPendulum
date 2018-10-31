function [x] = fnsimulate(xo,u_new,Horizon,dt,sigma)

global m;
global I;
global b;
global g;
global length;



x = xo;

% u_act = u_new(:,k) + (.5-rand(1))*sigma

for k = 1:(Horizon-1)
    
      Fx(1,1) = x(2,k);
      Fx(2,1) = u_new(:,k)/I - b/I*x(2,k) - m*g*length*sin(x(1,k))/I;
  
x(:,k+1) = x(:,k) + Fx * dt;
end