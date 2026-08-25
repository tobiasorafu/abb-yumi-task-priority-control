function p_obs = moving_obstacles(t, p_obs0)
    % Returns the current numberOfObstacles x 3 obstacle positions at time t.
    % Each obstacle orbits/oscillates around its own base position p_obs0(k,:).
    % Edit the per-obstacle offsets below to change speed/shape/amplitude
    % of motion, or add more obstacles if numberOfObstacles grows.
    p_obs = p_obs0;

    % obstacle 1: oscillates back and forth along x
    p_obs(1,:) = p_obs0(1,:) + [0.05*sin(0.5*t), 0, 0];

    % obstacle 2: small circular motion in the y-z plane
    p_obs(2,:) = p_obs0(2,:) + [0, 0.05*cos(0.3*t), 0.05*sin(0.3*t)];

    % obstacle 3: small circular motion in the x-y plane
    p_obs(3,:) = p_obs0(3,:) + [0.04*cos(0.4*t), 0.04*sin(0.4*t), 0];
end
