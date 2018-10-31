%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Differential Dynamic Programming               %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Course: Robotics and Autonomy                  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%  
%%%%%%%%%%%%%%%%%%%%%  AE8803  Fall  2018                             %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%  Author: Evangelos Theodorou                    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;

%change the model of the system to the inverted pendulum
% X e R^(2x1), U e R^1
% X = [theta; thetaDot];
% u is just control

global m;
global I;
global b;
global g;
global length;
global m_act;
global I_act;
global b_act;
global length_act;
global g_act;
g_act = 10;



% Inverted pendulum parameters.
m = 1; 
b = .1;
length = 2;
I = m*length^2;
g = 9.81;

m_act = 2;
b_act = .2;
length_act = 2.4;
I_act = m*length^2;

%m_act = 1;
%b_act = .1;
%length_act = 2;


% Horizon 
Horizon = 150; % 1.5sec
% Number of Iterations
num_iter = 200;

% Discretization
dt = 0.01;



% Weight in the Control:
R = .1;

% Initial Configuration:
xo = zeros(2,1);

% Initial Control:
u_k = zeros(1,Horizon-1);



% Initial trajectory:
x_traj = zeros(2,Horizon);
 
%number of steps to go before recomputing trajectory.
step_reeval = 1;
x_act = [xo];

%Implement MPC 
for i = 1:Horizon
    %change the horizon as well.
    new_Horizon = Horizon - i;
    [x_traj, u_opt, L_opt] = fnDDP(xo, x_traj, num_iter, new_Horizon, u_k);

    %step the algorithm forward 1 step.
    sigma = 10;
    [xo] = fnsimulateStep(xo, u_opt, new_Horizon, dt, sigma);
    %[act_traj, u_act] = fnsimulateStoch(xo,u_opt,new_Horizon,dt,sigma, u_opt, L_opt,x_traj);
    
    if i == 1
        exp_traj = x_traj;
    end
    %adjust the trajectory.
    x_traj = x_traj(:,2:end);
    x_act = [x_act, xo];
end
%implement the stochastic disturbances. (or other disturbances)

% exp_length = length;
% length = length*.5;
% act_length = length;
% %if 1, use L to correct the control authority
% U_control = 1;
% [act_traj, u_act] = fnsimulateStoch(xo,u_new,Horizon,dt,sigma, U_control, L_k,x_traj);

   time(1)=0;
   for i= 2:Horizon
    time(i) =time(i-1) + dt;  
   end

      %%

   figure(1);
   subplot(2,2,1)
   hold on
   plot(time,x_traj(1,:),'linewidth',4);  
   plot(time,act_traj(1,:),'linewidth',4);
   plot(time,p_target(1,1)*ones(1,Horizon),'red','linewidth',4)
   title('Theta 1','fontsize',20); 
    xlabel('Time in sec','fontsize',20)
   hold off;
   legend('Expected traj','Actual Traj','Target')
   grid;
   
   
    subplot(2,2,2);hold on;
   plot(time,x_traj(2,:),'linewidth',4); 
   plot(time,act_traj(2,:),'linewidth',4);
   plot(time,p_target(2,1)*ones(1,Horizon),'red','linewidth',4)
   title('Theta dot','fontsize',20);
    xlabel('Time in sec','fontsize',20)
    legend('Expected traj','Actual Traj','Target')
   hold off;
   grid;
   
   
    subplot(2,2,3);hold on
   plot(Cost,'linewidth',2);
    xlabel('Iterations','fontsize',20)
   title('Cost','fontsize',20);
   %save('DDP_Data');
   
    subplot(2,2,4);hold on
   plot(time(2:end), u_new,'linewidth',2); 
   plot(time(2:end), u_act,'linewidth',2);
    xlabel('Time (s)','fontsize',20)
   title('Control','fontsize',20);
   legend('Expected traj','Actual Traj')
   hold off
   %save('DDP_Data');
  
 %%
 %Animate the trajectory: (just the angle and x)
  iters = size(x_traj);
  iters = iters(2);
  figure 
  for m = 1:iters
       clf
       axis([-2*length,2*length,-2*length,2*length])
       line([0,exp_length* sin(x_traj(1,m))],[0,-exp_length*cos(x_traj(1,m))],'Color','Red')
       line([0,act_length* sin(act_traj(1,m))],[0,-act_length*cos(act_traj(1,m))],'Color','Blue')
    
       pause(dt)
       %pause(.5*dt)
  end