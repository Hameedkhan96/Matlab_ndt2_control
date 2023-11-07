function [A,B] = QuadrotorStateJacobianFcn(in1,in2)
%QuadrotorStateJacobianFcn
%    [A,B] = QuadrotorStateJacobianFcn(IN1,IN2)

%    This function was generated by the Symbolic Math Toolbox version 9.3.
%    20-Jul-2023 08:51:13

phi_t = in1(4,:);
phi_dot_t = in1(10,:);
psi_t = in1(6,:);
psi_dot_t = in1(12,:);
theta_t = in1(5,:);
theta_dot_t = in1(11,:);
u1 = in2(1,:);
u2 = in2(2,:);
u3 = in2(3,:);
u4 = in2(4,:);
t2 = cos(phi_t);
t3 = cos(psi_t);
t4 = cos(theta_t);
t5 = sin(phi_t);
t6 = sin(psi_t);
t7 = sin(theta_t);
t8 = phi_t.*2.0;
t9 = psi_dot_t.^2;
t10 = theta_t.*2.0;
t11 = theta_dot_t.^2;
t21 = u1+u2+u3+u4;
t12 = cos(t8);
t13 = t2.^2;
t14 = cos(t10);
t15 = t4.^2;
t16 = t4.^3;
t17 = sin(t8);
t18 = t5.^2;
t19 = sin(t10);
t20 = t7.^2;
t22 = t4.*6.0e+1;
t23 = 1.0./t4;
t26 = t7.*9.2e+1;
t27 = t7.*1.15e+2;
t32 = (t2.*t4)./2.0;
t33 = (t3.*t5)./2.0;
t34 = (t5.*t6)./2.0;
t41 = t2.*t4.*t5.*5.5e+1;
t42 = t2.*t5.*t7.*5.5e+1;
t47 = (t2.*t3.*t7)./2.0;
t50 = (t2.*t6.*t7)./2.0;
t24 = 1.0./t15;
t25 = 1.0./t16;
t28 = -t26;
t29 = t13.*4.4e+1;
t30 = t13.*5.5e+1;
t31 = t17.*2.2e+1;
t37 = -t33;
t43 = t7.*t13.*-4.4e+1;
t44 = t7.*t13.*-5.5e+1;
t52 = t7.*t41;
t53 = t4.*t13.*theta_dot_t.*5.06e+2;
t54 = t4.*t13.*u3.*-5.5e+1;
t59 = phi_dot_t.*psi_dot_t.*t13.*t15.*5.06e+2;
t60 = psi_dot_t.*t4.*t7.*t13.*theta_dot_t.*-5.06e+2;
t62 = t7.*t9.*t13.*t15.*5.06e+2;
t63 = t34+t47;
t35 = -t29;
t36 = -t30;
t38 = t4.*t30;
t39 = t7.*t29;
t40 = t7.*t30;
t51 = t15.*t30;
t55 = t44.*u4;
t56 = phi_dot_t.*t53;
t57 = t7.*t53;
t61 = -t59;
t64 = t37+t50;
t45 = t38.*u1;
t46 = t38.*u3;
t48 = t40.*u2;
et1 = (t24.*(t4.*u1.*-9.2e+1+t4.*u2.*9.2e+1-t4.*u3.*9.2e+1+t4.*u4.*9.2e+1+phi_dot_t.*t14.*theta_dot_t.*4.6e+1-psi_dot_t.*t7.*theta_dot_t.*1.058e+3-t4.*t13.*u2.*4.4e+1-t4.*t13.*u4.*4.4e+1+t4.*t29.*u1+t4.*t29.*u3+phi_dot_t.*t13.*t15.*theta_dot_t.*5.06e+2-phi_dot_t.*t13.*t20.*theta_dot_t.*5.06e+2+psi_dot_t.*t7.*t13.*theta_dot_t.*5.06e+2-t2.*t5.*t15.*u1.*5.5e+1+t2.*t5.*t15.*u3.*5.5e+1+t4.*t7.*t13.*u2.*1.1e+2+t2.*t5.*t20.*u1.*5.5e+1-t4.*t7.*t13.*u4.*1.1e+2-t2.*t5.*t20.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t16.*5.06e+2+psi_dot_t.*t7.*t13.*t15.*theta_dot_t.*1.518e+3+t2.*t4.*t5.*t7.*t9.*1.012e+3-t2.*t4.*t5.*t7.*t11.*1.012e+3-phi_dot_t.*psi_dot_t.*t2.*t4.*t5.*t20.*1.012e+3))./5.52e+2;
et2 = (t7.*t25.*(u2.*-1.15e+2+u4.*1.15e+2+t7.*t56-t7.*u1.*9.2e+1-t7.*u3.*9.2e+1-t13.*u4.*5.5e+1+t26.*u2+t26.*u4+t30.*u2+t39.*u1+t39.*u3+t43.*u2+t43.*u4+t51.*u4+t52.*u3+phi_dot_t.*t19.*theta_dot_t.*2.3e+1+psi_dot_t.*t4.*theta_dot_t.*1.058e+3-t13.*t15.*u2.*5.5e+1-psi_dot_t.*t4.*t13.*theta_dot_t.*5.06e+2-psi_dot_t.*t13.*t16.*theta_dot_t.*5.06e+2-t2.*t5.*t9.*t15.*5.06e+2+t2.*t5.*t11.*t15.*5.06e+2-t2.*t4.*t5.*t7.*u1.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2))./2.76e+2;
et3 = t24.*(t4.*u2.*1.15e+2-t4.*u4.*1.15e+2+t38.*u4+t42.*u3+phi_dot_t.*t7.*theta_dot_t.*4.6e+1-psi_dot_t.*t14.*theta_dot_t.*1.058e+3-t4.*t13.*u2.*5.5e+1+phi_dot_t.*t7.*t13.*theta_dot_t.*5.06e+2+psi_dot_t.*t13.*t15.*theta_dot_t.*5.06e+2-psi_dot_t.*t13.*t20.*theta_dot_t.*5.06e+2+t2.*t5.*t9.*t16.*5.06e+2-t2.*t5.*t7.*u1.*5.5e+1-t2.*t4.*t5.*t9.*t20.*1.012e+3+phi_dot_t.*psi_dot_t.*t2.*t4.*t5.*t7.*1.012e+3).*(-1.0./5.52e+2);
et4 = (t7.*t25.*(t48+t55+t56+t60-u1.*9.2e+1+u2.*9.2e+1-u3.*9.2e+1+u4.*9.2e+1-t7.*u2.*1.15e+2-t13.*u2.*4.4e+1-t13.*u4.*4.4e+1+t29.*u1+t27.*u4+t29.*u3+t41.*u3+phi_dot_t.*t4.*theta_dot_t.*4.6e+1+psi_dot_t.*t19.*theta_dot_t.*5.29e+2-t2.*t4.*t5.*u1.*5.5e+1+phi_dot_t.*psi_dot_t.*t2.*t5.*t15.*5.06e+2-t2.*t5.*t7.*t9.*t15.*5.06e+2))./2.76e+2;
mt1 = [0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,(t21.*(t2.*t6-t3.*t5.*t7))./2.0,t21.*(t2.*t3+t5.*t6.*t7).*(-1.0./2.0),t4.*t5.*t21.*(-1.0./2.0)];
mt2 = [(t24.*(t7.*t46+t7.*t59-t9.*t13.*t15.*5.06e+2+t11.*t13.*t15.*5.06e+2+t9.*t15.*t18.*5.06e+2-t11.*t15.*t18.*5.06e+2-t2.*t5.*u2.*1.1e+2+t2.*t5.*u4.*1.1e+2+t4.*t44.*u1-t2.*t5.*t7.*u1.*8.8e+1+t2.*t5.*t7.*u2.*8.8e+1-t2.*t5.*t7.*u3.*8.8e+1+t2.*t5.*t7.*u4.*8.8e+1+t2.*t5.*t15.*u2.*1.1e+2-t2.*t5.*t15.*u4.*1.1e+2+t4.*t7.*t18.*u1.*5.5e+1-t4.*t7.*t18.*u3.*5.5e+1-phi_dot_t.*psi_dot_t.*t7.*t15.*t18.*5.06e+2+psi_dot_t.*t2.*t4.*t5.*theta_dot_t.*1.012e+3+psi_dot_t.*t2.*t5.*t16.*theta_dot_t.*1.012e+3-phi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*1.012e+3))./5.52e+2];
mt3 = [t23.*(t48+t55+t56+t60+t12.*u1.*4.4e+1-t12.*u2.*4.4e+1+t12.*u3.*4.4e+1-t12.*u4.*4.4e+1-t7.*t18.*u2.*5.5e+1+t7.*t18.*u4.*5.5e+1-phi_dot_t.*t4.*t18.*theta_dot_t.*5.06e+2-t2.*t4.*t5.*u1.*1.1e+2+t2.*t4.*t5.*u3.*1.1e+2+phi_dot_t.*psi_dot_t.*t2.*t5.*t15.*1.012e+3+psi_dot_t.*t4.*t7.*t18.*theta_dot_t.*5.06e+2-t2.*t5.*t7.*t9.*t15.*1.012e+3).*(-1.0./5.52e+2)];
mt4 = [t24.*(t45+t54+t61+t62+t2.*t5.*u1.*8.8e+1-t2.*t5.*u2.*8.8e+1+t2.*t5.*u3.*8.8e+1-t2.*t5.*u4.*8.8e+1-t4.*t18.*u1.*5.5e+1+t4.*t18.*u3.*5.5e+1+phi_dot_t.*psi_dot_t.*t15.*t18.*5.06e+2-t7.*t9.*t15.*t18.*5.06e+2+t2.*t5.*t7.*u2.*1.1e+2-t2.*t5.*t7.*u4.*1.1e+2+phi_dot_t.*t2.*t4.*t5.*theta_dot_t.*1.012e+3-psi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*1.012e+3).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t3.*t21.*t32,t6.*t21.*t32,t2.*t7.*t21.*(-1.0./2.0),et1+et2];
mt5 = [(t23.*(t7.*u1.*6.0e+1-t7.*u3.*6.0e+1+t40.*u1+t41.*u4+t44.*u3-t9.*t13.*t16.*5.06e+2+phi_dot_t.*psi_dot_t.*t4.*t7.*1.104e+3+t4.*t9.*t13.*t20.*1.012e+3-t2.*t4.*t5.*u2.*5.5e+1-phi_dot_t.*psi_dot_t.*t4.*t7.*t13.*1.012e+3+phi_dot_t.*t2.*t5.*t7.*theta_dot_t.*5.06e+2+psi_dot_t.*t2.*t5.*t15.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t5.*t20.*theta_dot_t.*5.06e+2))./5.52e+2-(t7.*t24.*(t45+t54+t61+t62-t4.*u3.*6.0e+1-t17.*u2.*2.2e+1-t17.*u4.*2.2e+1+t22.*u1+t31.*u1+t31.*u3+t42.*u2+phi_dot_t.*psi_dot_t.*t15.*5.52e+2-t2.*t5.*t7.*u4.*5.5e+1+phi_dot_t.*t2.*t4.*t5.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t4.*t5.*t7.*theta_dot_t.*5.06e+2))./5.52e+2,et3+et4,0.0,0.0,0.0,0.0,0.0,0.0];
mt6 = [(t21.*(t3.*t5-t2.*t6.*t7))./2.0,(t21.*(t5.*t6+t2.*t3.*t7))./2.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0,0.0,(t24.*(t57+t19.*theta_dot_t.*2.3e+1+psi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2))./5.52e+2,t23.*(psi_dot_t.*t15.*5.52e+2-psi_dot_t.*t13.*t15.*5.06e+2+t2.*t4.*t5.*theta_dot_t.*5.06e+2).*(-1.0./5.52e+2),(t24.*(t53+t4.*theta_dot_t.*4.6e+1+psi_dot_t.*t2.*t5.*t15.*5.06e+2))./5.52e+2,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,0.0];
mt7 = [(t24.*(phi_dot_t.*t19.*2.3e+1+psi_dot_t.*t4.*1.058e+3-psi_dot_t.*t4.*t13.*5.06e+2-psi_dot_t.*t13.*t16.*5.06e+2+phi_dot_t.*t4.*t7.*t13.*5.06e+2+t2.*t5.*t15.*theta_dot_t.*1.012e+3))./5.52e+2,t23.*(phi_dot_t.*t2.*t4.*t5.*5.06e+2-psi_dot_t.*t2.*t4.*t5.*t7.*5.06e+2).*(-1.0./5.52e+2),(t24.*(phi_dot_t.*t4.*4.6e+1+psi_dot_t.*t19.*5.29e+2+phi_dot_t.*t4.*t13.*5.06e+2-psi_dot_t.*t4.*t7.*t13.*5.06e+2))./5.52e+2,0.0,0.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,t24.*(t53-t4.*theta_dot_t.*1.058e+3+t13.*t16.*theta_dot_t.*5.06e+2+psi_dot_t.*t2.*t5.*t15.*1.012e+3-phi_dot_t.*t2.*t5.*t7.*t15.*5.06e+2).*(-1.0./5.52e+2)];
mt8 = [t23.*(phi_dot_t.*t15.*5.52e+2-phi_dot_t.*t13.*t15.*5.06e+2+psi_dot_t.*t7.*t13.*t15.*1.012e+3-t2.*t4.*t5.*t7.*theta_dot_t.*5.06e+2).*(-1.0./5.52e+2),(t24.*(t19.*theta_dot_t.*5.29e+2+phi_dot_t.*t2.*t5.*t15.*5.06e+2-t4.*t7.*t13.*theta_dot_t.*5.06e+2-psi_dot_t.*t2.*t5.*t7.*t15.*1.012e+3))./5.52e+2];
A = reshape([mt1,mt2,mt3,mt4,mt5,mt6,mt7,mt8],12,12);
if nargout > 1
    mt9 = [0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,t24.*(t26+t43+t52).*(-1.0./5.52e+2),t23.*(t22+t31+t38).*(-1.0./5.52e+2),t24.*(t35+t41+9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,t24.*(t28+t36+t39+t51+1.15e+2).*(-1.0./5.52e+2),(t23.*(t31-t42))./5.52e+2,t24.*(t27+t29+t44-9.2e+1).*(-1.0./5.52e+2),0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,(t24.*(t28+t39+t52))./5.52e+2,(t23.*(t22-t31+t38))./5.52e+2,(t24.*(t29+t41-9.2e+1))./5.52e+2,0.0,0.0,0.0,0.0,0.0,0.0,t63,t64,t32,(t24.*(t26+t36+t43+t51+1.15e+2))./5.52e+2,(t23.*(t31+t42))./5.52e+2];
    mt10 = [(t24.*(t27+t35+t44+9.2e+1))./5.52e+2];
    B = reshape([mt9,mt10],12,4);
end
end