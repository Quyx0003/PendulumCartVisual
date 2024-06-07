function X_next = Euler_2nd_order(Xddot, dt, u, M, m, g, l, c, b, I)
    % Extract position and velocity from input
    x = Xddot(1:2);
    z = Xddot(3:4);
    
    % Compute derivative of position (velocity)
    dxdt = z;

    % Compute derivative of velocity (acceleration)
    dzdt = InvertedPendulum(x, z, u, M, m, g, l, c, b, I);
    
    % Update position using Euler integration
    x_update = x + dt * dxdt;

    % Update velocity using Euler integration
    z_update = z + dt * dzdt;

    % Combine updated position and velocity into output state vector
    X_next = [x_update; z_update];
end
