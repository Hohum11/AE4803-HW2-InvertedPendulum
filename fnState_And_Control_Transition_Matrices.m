function [A,B] = fnState_And_Control_Transition_Matrices(x,u,du,dt)

global m;
global I;
global b;
global g;
global length;

x1 = x(1,1); %theta
x2 = x(2,1); %theta dot

u1 = u;

A = zeros(2,2);


B = zeros(2,1);

A(1,2) = 1;
A(2,1) = -m*g*length*cos(x1)/I;
A(2,2) = -b/I;

 
B(2,1) = 1/I;




