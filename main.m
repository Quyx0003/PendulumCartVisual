clc
clear
close all

%% Select control mode: "PID" or "LQR"
cd = "LQR";

%% Parameters
mc = 0.5;                               % Mass of cart
mp = 0.084;                             % Mass of pendulum
Ip = 0.0008575;                         % MOI of Pendulum
l = 0.175;                              % COM of Pendulum
g = 9.81;                               % Gravity Constant
bc = 5;                                 % Linear Damping Conveyor Belt
bp = 0.0012;                            % Rotational Damping Pendulum

%% Define the state-space matrices
A = [0, 0, 1, 0;
     0, 0, 0, 1;
     0, (g * l * l * mp * mp) / (mc * mp * l * l + Ip * mc + Ip * mp), -(bc * (mp * l * l + Ip)) / (mc * mp * l * l + Ip * mc + Ip * mp), -(bp * l * mp) / (mc * mp * l * l + Ip * mc + Ip * mp);
     0, (g * l * mp + (g * l * l * l * mp * mp * mp) / ((mc + mp) * (mp * l * l + Ip))) / (mp * l * l + Ip), -(bc * l * mp) / ((mc + mp) * (mp * l * l + Ip)), -(bp + (bp * l * l * mp * mp) / ((mc + mp) * (mp * l * l + Ip))) / (mp * l * l + Ip)];

B = [0;
     0;
     (mp * l * l + Ip) / (mc * mp * l * l + Ip * mc + Ip * mp);
     (l * mp) / ((mc + mp) * (mp * l * l + Ip))];

%% Calculation of PID gain
Kp = 125;                               % Proportional gain
Ki = 635;                               % Integral gain
Kd = 10.4;                              % Derivative gain

%% Calculation of LQR gain
Q = diag([15, 27.7, 1, 1]);
R = 0.001;
lqr = lqr(A, B, Q, R); 

%% Close Loop Control simulation
Ts = 0.001;                             % Sample time 
Tf = 10;                                % Total simulation time
X_des = [0; pi; 0; 0];                  % Desired state 
Epot = 2 * mp * g * l;                  % Desired energy
x_constraint = 0.5;                     
k_swing = 5;

% Initializations
errorIntegral = 0;                      % Initialize error integral 
prev_error = 0;                         % Initialize previous error
Xp = zeros(ceil(Tf / Ts), 4);           % Initialize state array
u0 = 0;                                 % Initialize control input
i = 0;                                  % Initialize loop counter
X0 = [0; 1 * (pi / 180); 0; 0];         % Initial state

% Simulation loop
for k = 0:Ts:Tf
    i = i + 1;

    % Update system state using Euler integration
    new_state = Euler_2nd_order(X0, Ts, u0, mc, mp, g, l, bc, bp, Ip);
    
    % Handle pendulum angle wrapping
    if new_state(2) < 0
       th = 2 * pi - abs(new_state(2));
       updated_state = [new_state(1); th; new_state(3); new_state(4)];
    else
       updated_state = new_state;
    end
    
    % Store updated state
    Xp(i, :) = updated_state;

    % Update initial state
    X0 = new_state; 

    % Extract state variables
    x = new_state(1);
    theta = new_state(2);
    x_dot = new_state(3);
    dtheta = new_state(4);
    
    % Calculate energy
    E = (1 / 2) * (mc * x_dot * x_dot + mp * (x_dot * x_dot + 2 * x_dot * l * dtheta * cos(theta) + l * l * dtheta * dtheta) + Ip * dtheta * dtheta) - mp * g * l * cos(theta);
    accel = k_swing * (E - Epot) * sign(dtheta * cos(theta));

    % Calculate swing-Up Control
    x_penalty = 10 * tanh(5 * (x - x_constraint)) + 10 * tanh(5 * (x + x_constraint));
    swing = accel * (mc + mp) + bc * x_dot - mp * l * (-bp * dtheta + accel + mp * l * g * theta) / (Ip + mp * l * l - mp * l) - x_penalty;

    % PID control Design
    e = X_des(2) - updated_state(2);
    de = (e - prev_error) / Ts;
    prev_error = e;
    errorIntegral = errorIntegral + e * Ts;
    PIDcd = Kp * e + Ki * errorIntegral + Kd * de;

    % LQR control Design 
    LQRcd = lqr * (X_des - updated_state); 

    % Update control signal based on mode (PID or LQR)
    if (abs(X_des(2) - updated_state(2))) * (180 / pi) <= 30 
        if cd == "PID"
            u0 = PIDcd;
        elseif cd == "LQR" 
            u0 = LQRcd;
        end
    else                                   
        u0 = swing;  
    end    
end

%% Animation plot of Inverted Pendulum System
hf = figure();
for i = 1:20:length(Xp)
   Visual(Xp(i, 1), Xp(i, 2));
   pause(Ts);
   hold off
end
