%Control of Quadrotor Using Nonlinear Model Predictive Control

%Call the script getQuadrotorDynamicsAndJacobian to generate and write to a file both the state and its Jacobians functions.

getQuadrotorDynamicsAndJacobian;


%Design Nonlinear Model Predictive Controller

%Create a nonlinear MPC object with 12 states, 12 outputs, and 4 inputs. By default, all the inputs are manipulated variables (MVs).
nx = 12;
ny = 12;
nu = 4;
nlmpcobj = nlmpc(nx, ny, nu);

%Specify the prediction model state function using the function name.
nlmpcobj.Model.StateFcn = "QuadrotorStateFcn";

%It is best practice to provide an analytical Jacobian for the prediction
% model. Doing so significantly improves simulation efficiency.

nlmpcobj.Jacobian.StateFcn = @QuadrotorStateJacobianFcn;

%Fix the random generator seed for reproducibility.
rng(0)

%To check that your prediction model functions for nlobj are valid, use validateFcns for a random point in the state-input space

validateFcns(nlmpcobj,rand(nx,1),rand(nu,1));

%Parameter Defining
Ts = 0.1;
p = 18;
m = 2;
nlmpcobj.Ts = Ts;
nlmpcobj.PredictionHorizon = p;
nlmpcobj.ControlHorizon = m;

%Contriants : 
% %Limit all four control inputs to be in the range [0,10]. Also limit control
% input change rates to the range [-2,2] to prevent abrupt and rough movements.
nlmpcobj.MV = struct(Min={0;0;0;0},Max={10;10;10;10}, RateMin={-2;-2;-2;-2},RateMax={2;2;2;2});

%%%%cost function
nlmpcobj.Weights.OutputVariables = [1 1 1 1 1 1 0 0 0 0 0 0];
%To prioritize OV targets, set the average MV tracking priority lower than the average OV tracking priority.
nlmpcobj.Weights.ManipulatedVariables = [0.1 0.1 0.1 0.1];
%Also, penalize overly aggressive control actions by specifying tuning weights for the MV rates of change.
nlmpcobj.Weights.ManipulatedVariablesRate = [0.1 0.1 0.1 0.1];


%%%Closed-Loop Simulation
% Specify the initial conditions
x = [7;-10;0;0;0;0;0;0;0;0;0;0];

% Nominal control target (average to keep quadrotor floating)
nloptions = nlmpcmoveopt;
nloptions.MVTarget = [4.9 4.9 4.9 4.9]; 
mv = nloptions.MVTarget;

%Simulate the closed-loop system using the nlmpcmove function, specifying simulation options using an nlmpcmove object.


% Simulation duration in seconds
Duration = 20;

% Display waitbar to show simulation progress
hbar = waitbar(0,"Simulation Progress");

% MV last value is part of the controller state
lastMV = mv;

% Store states for plotting purposes
xHistory = x';
uHistory = lastMV;

% Simulation loop
for k = 1:(Duration/Ts)

    % Set references for previewing
    t = linspace(k*Ts, (k+p-1)*Ts,p);
    yref = QuadrotorReferenceTrajectory(t);

    % Compute control move with reference previewing
    xk = xHistory(k,:);
    [uk,nloptions,info] = nlmpcmove(nlmpcobj,xk,lastMV,yref',[],nloptions);

    % Store control move
    uHistory(k+1,:) = uk';
    lastMV = uk;

    % Simulate quadrotor for the next control interval (MVs = uk) 
    ODEFUN = @(t,xk) QuadrotorStateFcn(xk,uk);
    [TOUT,XOUT] = ode45(ODEFUN,[0 Ts], xHistory(k,:)');

    % Update quadrotor state
    xHistory(k+1,:) = XOUT(end,:);

    % Update waitbar
    waitbar(k*Ts/Duration,hbar);
end

% Close waitbar 
close(hbar)


%%Visualization and Results
plotQuadrotorTrajectory;

animateQuadrotorTrajectory;







