function [x, u_act] = fnsimulateStep(xo, u_opt, new_Horizon, dt, sigma)
global m_act;
global I_act;
global b_act;
global g_act;
global length_act;



x = xo;


u_act = u_opt + (.5-rand(1))*sigma;
k = 1;
Fx(1,1) = x(2,k);
Fx(2,1) = u_act(k)/I_act - b_act/I_act*x(2,k) - m_act*g_act*length_act*sin(x(1,k))/I_act;
  
x(:,k+1) = x(:,k) + Fx * dt;
x = x(:,2);


end