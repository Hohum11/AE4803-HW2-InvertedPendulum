function [x, u_exerted] = fnsimulateStoch(xo,u_new,Horizon,dt,sigma, U_control, L_k,x_traj)

global m_act;
global I_act;
global b_act;
global g_act;
global length_act;



x = xo;


for k = 1:(Horizon-1)
    if U_control
        u_exerted(:,k) = u_new(:,k) + L_k(:,:,k)*(x(:,k)-x_traj(:,k));
    else 
        u_exerted(:,k) = u_new(:,k);
    end
      u_act = u_exerted(:,k)+ (.5-rand(1))*sigma;
      Fx(1,1) = x(2,k);
      Fx(2,1) = u_act/I_act - b_act/I_act*x(2,k) - m_act*g_act*length_act*sin(x(1,k))/I_act;
  
x(:,k+1) = x(:,k) + Fx * dt;
end