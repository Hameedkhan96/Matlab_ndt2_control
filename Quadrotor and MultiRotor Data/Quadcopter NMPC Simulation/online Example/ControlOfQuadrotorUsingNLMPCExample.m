
getQuadrotorDynamicsAndJacobian;

nx = 12;
ny = 12;
nu = 4;
nlobj = nlmpc(nx, ny, nu);

nlobj.Model.StateFcn = "QuadrotorStateFcn";
nlobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;

rng(0)
validateFcns(nlobj,rand(nx,1),rand(nu,1));

Ts = 0.1;
p = 18;
m = 2;
nlobj.Ts = Ts;
nlobj.PredictionHorizon = p;
nlobj.ControlHorizon = m;

nlobj.MV = struct('Min',{0;0;0;0},'Max',{12;12;12;12});

nlobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];

nlobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];

nlobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];

% Specify the initial conditions
x = [7;-10;0;0;0;0;0;0;0;0;0;0];
% Nominal control that keeps the quadrotor floating
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [4.9 4.9 4.9 4.9]; 
mv = nloptions.MVTarget;

Duration = 20;
hbar = waitbar(0,'Simulation Progress');
xHistory = x';
lastMV = mv;
uHistory = lastMV;
for k = 1:(Duration/Ts)
    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);
    % Compute the control moves with reference previewing.
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlobj,xk,lastMV,yref',[],nloptions);
    uHistory(k+1,:) = uk';
    lastMV = uk;
    % Update states.
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,YOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');
    xHistory(k+1,:) = YOUT(end,:);
    waitbar(k*Ts/Duration,hbar);
end
close(hbar)

plotQuadrotorTrajectory


%animateQuadrotorTrajectory;