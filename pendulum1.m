% Jacobian Linearization of simple inverted pendulum
clear; close all; clc;

% Define some symbols
syms s x1 x2  u  x_punkt x1_punkt x2_punkt;

p = 1;

% Nonlinear dynamics
% x1 is angle of pendulum from downward vertical
% x2 is angular rate of pendulum
x1_punkt = x2;
x2_punkt = -sin(x1) + u;

% Ruhelage
xR = [pi; 0];

% Linearisierung
% x = [x1 x2];
% u = [u];
% n = 2;                                      %Variablen
% m = 1;                                      %Inputs
% p = 0;                                      %Outputs
% x_punkt = [x1_punkt x2_punkt]';
% linear_A = jacobian(x_punkt, x);
% linear_B = jacobian(x_punkt, u);
% linear_C = jacobian(x);
% 
% A= subs(linear_A,x',xR);
% 
% 
% B= subs(linear_B,x',xR);
% 
% 
% C= subs(linear_C,x',xR);
% 
% 
% D = zeros(p,m);
% sys = ss(A,B,C,D);

% x and u input vectors for Jacobian function
x = [x1;x2];
u = [u];

f = [x1_punkt; x2_punkt];
A = jacobian(f, x);
B = jacobian(f, u);
C = [1 0];
D = [0];

% Evaluate the Jacobian matrices at equilibrium points and
% define the unknown parameter p
x1 = xR(1);
x2 = xR(2);
p = 1;

A = eval(A);
B = eval(B);

sys = ss(A,B,C,D)

%% Stabilität des AP
eig(A)

%% Steuerbarkeit des AP
Qs = [B A*B]
RangQs = rank(Qs)

%% Beobachtbarkeit des AP
Qb = [C; C*A]
RangQb = rank(Qb)

%% Übertragungsfunktion
[NUM, DEN] = ss2tf(A, B, C ,D)
G_tf = tf(NUM, DEN)
G_star = inv(G_tf)

pole (G_tf)                                                    
zero (G_tf)

%%Filter
b = limit(((s^2 + 1)/(20*s^2))^0.5, s, inf)
lamda = 20^(1/2)/20       

%%
G_filter = tf(1, [0.16 0.8 1])

G_tf_stable = tf(1, [1 0 1])
G_star_stable = inv(G_tf_stable)
G_star_filter = G_star_stable*G_filter