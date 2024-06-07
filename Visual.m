function Visual(x, th)
    %% Define parameters
    W = 0.2;  % Width of the cart
    H = 0.08; % Height of the cart
    L = 0.35; % Length of the pendulum

    %% Position of the pendulum
    px = x + L * sin(th);
    py = 0 - L * cos(th);

    %% Plot elements
    % Plot the base line
    plot([-2 2],[0 0],'k','LineWidth', 2);
    hold on;
    % Plot the cart
    rectangle('Position', [x-W/2,0-H/2,W,H], 'Curvature', 0.1, 'FaceColor', [0 0 0], 'EdgeColor', [1 1 1]); 
    
    % Plot the pendulum
    plot([x px], [0 py], 'Color', [0.2 0.2 0.2], 'LineWidth', 2.5); 
    
    % Plot the circle at the pendulum end
    viscircles([px py], 0.02, 'Color', [0.2 0.2 0.2], 'LineWidth', 2.5); 
    
    % Plot the circle at the cart
    viscircles([x 0], 0.02, 'Color', 'w', 'LineWidth', 0.2); 
    
    %% Set plot properties
    xlabel('X (m)');            % Label for X-axis
    ylabel('Y (m)');            % Label for Y-axis
    title('Inverted Pendulum'); % Title of the plot
    axis(gca, 'equal');         % Set equal scaling for axes
    xlim([-2 2]);               % Set X-axis limits
    ylim([-0.8 1]);             % Set Y-axis limits
    grid on;                    % Display grid lines
end