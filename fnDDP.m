function [x_traj, u_opt, L_opt] = fnDDP (xo, x_traj, num_iter, Horizon, u_k, )

du_k = zeros(1,Horizon-1);
% Target: 
p_target(1,1) = pi;
p_target(2,1) = 0;


% Learning Rate:c
gamma = .1
R = 0.1;
dt = 0.01;

% Weight in Final State:
Q_f = zeros(2,2);
Q_f(1,1) = 100;
Q_f(2,2) = 10;
 
if Horizon <= 1
    x_traj = 0;
    u_opt = 0;
    L_opt = 0;
else

for k = 1:num_iter

%------------------------------------------------> Linearization of the dynamics
%------------------------------------------------> Quadratic Approximations of the cost function 
for  j = 1:(Horizon-1)
    
     [l0,l_x,l_xx,l_u,l_uu,l_ux] = fnCost(x_traj(:,j), u_k(:,j), j,R,dt);
      q0(j) = dt * l0;
      q_k(:,j) = dt * l_x;
      Q_k(:,:,j) = dt * l_xx;
      r_k(:,j) = dt * l_u;
      R_k(:,:,j) = dt * l_uu;
      P_k(:,:,j) = dt * l_ux; 
    
    [dfx,dfu] = fnState_And_Control_Transition_Matrices(x_traj(:,j),u_k(:,j),du_k(:,j),dt);
   
    A(:,:,j) = eye(2,2) + dfx * dt;
    B(:,:,j) = dfu * dt;  
end

%------------------------------------------------> Find the controls
Vxx(:,:,Horizon)= Q_f;
Vx(:,Horizon) = Q_f * (x_traj(:,Horizon) - p_target); 
V(Horizon) = 0.5 * (x_traj(:,Horizon) - p_target)' * Q_f * (x_traj(:,Horizon) - p_target); 


%------------------------------------------------> Backpropagation of the Value Function
for j = (Horizon-1):-1:1
   Q(:,j) = q0(j) + V(:,j+1);
   Qx(:,j) = q_k(:,j) + A(:,:,j)'*Vx(:,j+1);
   Qxx(:,:,j) = Q_k(:,:,j) + A(:,:,j)'*Vxx(:,:,j+1)*A(:,:,j);
   Qu(:,j) = r_k(:,j) + B(:,:,j)'*Vx(:,j+1);
   Quu(:,:,j) = R_k(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*B(:,:,j);
   Qux(:,:,j) = P_k(:,:,j) + B(:,:,j)'*Vxx(:,:,j+1)*A(:,:,j);

   l_k(:,j) = -inv(Quu(:,:,j))*Qu(:,j)';
   L_k(:,:,j) = -inv(Quu(:,:,j))*Qux(:,:,j);
   
   V(:,j) = Q(:,j) - .5 * Qu(:,j)'*inv(Quu(:,:,j))*Qu(:,j);
   Vx(:,j) = Qx(:,j)' - Qu(:,j)'*inv(Quu(:,:,j))*Qux(:,:,j);
   Vxx(:,:,j) = Qxx(:,:,j) - Qux(:,:,j)'*inv(Quu(:,:,j))*Qux(:,:,j);


end 


%----------------------------------------------> Find the controls
dx = zeros(2,1);
for i=1:(Horizon-1)    
   du = l_k(:,i) + L_k(:,:,i) * dx;
   dx = A(:,:,i) * dx + B(:,:,i) * du;  
   u_new(:,i) = u_k(:,i) + gamma * du;
end
u_k = u_new;


%---------------------------------------------> Simulation of the Nonlinear System
[x_traj] = fnsimulate(xo,u_new,Horizon,dt,0);
[Cost(:,k)] =  fnCostComputation(x_traj,u_k,p_target,dt,Q_f,R);
x1(k,:) = x_traj(1,:);
 

fprintf('iLQG Iteration %d,  Current Cost = %e \n',k,Cost(1,k));
 
 u_opt = u_k;
 L_opt = L_k(:,:,end);
 
end
end
