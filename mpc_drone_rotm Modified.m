%
% Copyright 2019 Gianluca Frison, Dimitris Kouzoupis, Robin Verschueren,
% Andrea Zanelli, Niels van Duijkeren, Jonathan Frey, Tommaso Sartor,
% Branimir Novoselnik, Rien Quirynen, Rezart Qelibari, Dang Doan,
% Jonas Koenemann, Yutao Chen, Tobias Schöls, Jonas Schlagenhauf, Moritz Diehl
%
% This file is part of acados.
%
% The 2-Clause BSD License
%
% Redistribution and use in source and binary forms, with or without
% modification, are permitted provided that the following conditions are met:
%
% 1. Redistributions of source code must retain the above copyright notice,
% this list of conditions and the following disclaimer.
%
% 2. Redistributions in binary form must reproduce the above copyright notice,
% this list of conditions and the following disclaimer in the documentation
% and/or other materials provided with the distribution.
%
% THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
% AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
% IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
% ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
% LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
% CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
% SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
% INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
% CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
% ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
% POSSIBILITY OF SUCH DAMAGE.;
%

function model = mpc_drone_rotm()

import casadi.*

%% system dimensions
nx = 24;
nu = 6;
np = 18;

%% system parameters
m = 6.3833;
g = 9.81;
J = [0.29, 0, 0; 0, 0.29, 0; 0, 0, 0.3527];
f_ext = zeros(3,1);
tau_ext = zeros(3,1);
%% named symbolic variables
% states
x = SX.sym('x'); 
y = SX.sym('y'); 
z = SX.sym('z');

R11 = SX.sym('R11'); 
R12 = SX.sym('R12');
R13 = SX.sym('R13');
R21 = SX.sym('R21'); 
R22 = SX.sym('R22');
R23 = SX.sym('R23');
R31 = SX.sym('R31'); 
R32 = SX.sym('R32');
R33 = SX.sym('R33');

vx = SX.sym('vx'); 
vy = SX.sym('vy'); 
vz = SX.sym('vz');

wx = SX.sym('wx'); 
wy = SX.sym('wy'); 
wz = SX.sym('wz');

fx = SX.sym('fx');
fy = SX.sym('fy');
fz = SX.sym('fz');

tx = SX.sym('tx');
ty = SX.sym('ty');
tz = SX.sym('tz');

% input
fxd = SX.sym('fxd');        
fyd = SX.sym('fyd');         
fzd = SX.sym('fzd');

tauxd = SX.sym('tauxd');
tauyd = SX.sym('tauyd');
tauzd = SX.sym('tauzd'); 
%param
p = SX.sym('p', np, 1); 
xd_des = p(1); 
yd_des = p(2); 
zd_des = p(3); 
wx_des = p(4); 
wy_des = p(5); 
wz_des = p(6); 
R11_des = p(7); 
R12_des = p(8); 
R13_des = p(9);
R21_des = p(10); 
R22_des = p(11); 
R23_des = p(12);
R31_des = p(13); 
R32_des = p(14); 
R33_des = p(15);
x_des = p(16); 
y_des = p(17); 
z_des = p(18); 
%% (unnamed) symbolic variables
% sym_x = vertcat(x, y, z, qw, qx, qy, qz, vx, vy, vz, wx, wy, wz, fx, fy, fz, tx, ty, tz);
sym_x = vertcat(vx, vy, vz, wx, wy, wz, R11, R12, R13, R21, R22, R23, R31, R32, R33, x, y, z, fx, fy, fz, tx, ty, tz);
sym_xdot = SX.sym('xdot', nx, 1);
sym_u = vertcat(fxd, fyd, fzd, tauxd, tauyd, tauzd);

%% dynamics

p_lin = [x,y,z]';
v = [vx,vy,vz]';
w = [wx,wy,wz]';
F = [fx,fy,fz]';
tau = [tx,ty,tz]';

skew_w =  [ 0, -wz,  wy;
           wz,   0, -wx; 
          -wy,  wx,   0];
skew_Jw =  [         0,   -J(3,3)*wz,  J(2,2)*wy;
             J(3,3)*wz,            0, -J(1,1)*wx; 
            -J(2,2)*wy,    J(1,1)*wx,         0];

Fd = [fxd;fyd;fzd];
taud = [tauxd;tauyd;tauzd];

R_mat = [ R11, R12, R13; R21, R22, R23; R31, R32, R33];
% linear positon dynamics
pos_dyn = R_mat*v;
% attitude dynamics
% att_dyn = [cos(pitch), -sin(pitch)*cos(roll), -cos(pitch)*sin(roll); 0, cos(pitch), -sin(pitch); sin(pitch)/cos(roll), cos(pitch)/cos(roll), 0]*w;
att_dyn = R_mat*skew_w;
% uav dynamics

% [xd_dot(1);xd_dot(2);xd_dot(3)]
expr_f_expl1 = vertcat( (m*eye(3,3)) \ (F - m*skew_w*v + m*g*[0;0;1]) ); % dinamica (parte lineare) sec ordine del drone classica senza disturbi esterni (forza di contatto/attritto ecc)
% [wxd_dot(1);wxd_dot(2);wxd_dot(3)]
expr_f_expl2 = vertcat( expr_f_expl1, J \ (tau + skew_Jw*w) );          % dinamica (parte angolare) sec ordine del drone classica senza disturbi esterni (forza di contatto/attritto ecc)
% [x_dot(1);x_dot(2);x_dot(3); q_dot(1);q_dot(2);q_dot(3);q_dot(4)] 
expr_f_expl3 = vertcat( expr_f_expl2, att_dyn(1,1), att_dyn(1,2), att_dyn(1,3), att_dyn(2,1), att_dyn(2,2), att_dyn(2,3), att_dyn(3,1), att_dyn(3,2), att_dyn(3,3), pos_dyn );                 % evoluzione posizione lineare e angolare
% [F_dot(1);F_dot(2);F_dot(3); tau_d(1);tau_d(2);tau_d(3)] 
expr_f_expl  = vertcat( expr_f_expl3, Fd, taud );                         % dinamica degli ingressi di controllo

% dinamica in forma implicita TC
expr_f_impl = expr_f_expl - sym_xdot; 
%% constraints
expr_h = sym_u;                                             % variabile da vincolare (ub e lb in questo caso) 

%% NL constraints
     
%% cost
W_x = diag([[600, 600, 1750], [600, 600, 1000], [750, 750, 1250], [1000, 1000, 2000], [0.1, 0.1, 0.1], [0.1, 0.1, 0.1]]);  % Matrice Q della cifra di merito
W_u = diag([10, 10, 10, 10, 10, 10]);                                                                                       % Matrice R della cifra di merito

R_mat = [ sym_x(7), sym_x(8), sym_x(9); sym_x(10), sym_x(11), sym_x(12); sym_x(13), sym_x(14), sym_x(15)];
R_des = [ R11_des, R12_des, R13_des; R21_des, R22_des, R23_des; R31_des, R32_des, R33_des];

e_vel = [sym_x(1)-xd_des; sym_x(2)-yd_des; sym_x(3)-zd_des];
% e_w =   [sym_x(4)-wx_des; sym_x(5)-wy_des; sym_x(6)-wz_des];
e_w = [sym_x(4);sym_x(5);sym_x(6)] - R_mat' * R_des * [wx_des;wy_des;wz_des]; %%%%%%%%%%%%%%%%%5
e_pos = [sym_x(16)-x_des; sym_x(17)-y_des; sym_x(18)-z_des];

angle_error_matrix = 0.5 * (R_des'* R_mat - R_mat' * R_des);
angle_error = [angle_error_matrix(3, 2), angle_error_matrix(1,3), angle_error_matrix(2, 1)]';
expr_ext_cost_e = [e_vel; e_w; angle_error; e_pos; sym_x(19:24)]'*W_x*[e_vel; e_w; angle_error; e_pos; sym_x(19:24)];
% expr_ext_cost_e = sym_x'* W_x * sym_x;                                                                      % costo corrente stato
expr_ext_cost = expr_ext_cost_e + sym_u' * W_u * sym_u;                                                     % lagrangiana allo stato corrente
% nonlinear least sqares
cost_expr_y = vertcat(sym_x, sym_u);
W = blkdiag(W_x, W_u);
model.cost_expr_y_e = sym_x;
model.W_e = W_x;


%% populate structure
model.nx = nx;
model.nu = nu;
model.np = np;
model.sym_x = sym_x;
model.sym_xdot = sym_xdot;
model.sym_u = sym_u;
model.sym_p = p;
model.expr_f_expl = expr_f_expl;
model.expr_f_impl = expr_f_impl;
% model.expr_phi = expr_phi;
model.expr_h = expr_h;
model.expr_ext_cost = expr_ext_cost;
model.expr_ext_cost_e = expr_ext_cost_e;
% model.constr_expr_h = h_expr;
model.cost_expr_y = cost_expr_y;
model.W = W;

end
