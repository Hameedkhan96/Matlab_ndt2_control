clc;
close all;
% sim('Copy_of_Quadcopter_Control_PID_Block_Updated.slx')
 sim('Quadcopter_Control_PID_Controller_Block_NAveed_Bhai.slx')
subplot(221)
z_ref=out.Ref_z(2)*ones(size(out.z(:,1)));
plot(out.z(:,1),z_ref);
hold on
plot(out.z(:,1),out.z(:,2))
title(['\fontsize{12}z'])

subplot(222)
plot(out.Ref_phi(:,1),out.Ref_phi(:,2));
hold on;
plot(out.phi(:,1),out.phi(:,2))
title(['\fontsize{12}phi(Degree)'])

subplot(223)
plot(out.Ref_theta(:,1),out.Ref_theta(:,2));
hold on;
plot(out.theta(:,1),out.theta(:,2))
title(['\fontsize{12}theta(Degree)'])

subplot(224)
plot(out.Ref_psi(:,1),out.Ref_psi(:,2));
hold on;
plot(out.psi(:,1),out.psi(:,2))
title(['\fontsize{12}psi(Degree)'])

