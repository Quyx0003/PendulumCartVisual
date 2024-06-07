function dxdt = InvertedPendulum(X, X_dot, u, M, m, g, l, c, b, I)
    %% Extract states from input vectors
    theta       = X(2);             % Pendulum angle
    x_dot       = X_dot(1);         % Cart velocity
    theta_dot   = X_dot(2);         % Pendulum angular velocity
    F           = u;                % Control force input             

    %% Compute dxdt
    % Compute acceleration of cart (x_ddot)
    x_ddot = -(F - c * x_dot + l * m * (theta_dot * theta_dot * sin(theta) + (cos(theta) * (b * theta_dot + g * l * m * sin(theta))) / (m * l * l + I))) / ((M + m) * (l * m * cos(theta) * l * m * cos(theta) / ((M + m) * (m * l * l + I)) - 1));
    
    % Compute angular acceleration of pendulum (theta_ddot)
    theta_ddot = (b * theta_dot + l * m * (g * sin(theta) + (cos(theta) * (l * m * sin(theta) * theta_dot * theta_dot + F - c * x_dot)) / (M + m))) / ((m * l * l + I) * (l * m * cos(theta) * l * m * cos(theta) / ((M + m) * (m * l * l + I)) - 1));

    % Combine accelerations into dxdt vector
    dxdt = [x_ddot; theta_ddot];
end