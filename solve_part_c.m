function partC = solve_part_c(controller, params, opts, X0)

    if nargin < 3
        opts = struct();
    end
    if nargin < 4
        X0 = [0 0 0 0 0];
    end

    Ts            = get_option(opts, 'Ts', 0.001);
    sim_duration  = get_option(opts, 'sim_duration', 20);
    Vel           = get_option(opts, 'Vel', 8);
    waypoint_file = get_option(opts, 'waypoint_file', 1);
    voltage_limit = get_option(opts, 'voltage_limit', 12);
    steer_lim_rad = deg2rad(get_option(opts, 'steering_limit_deg', 20));

    steps = floor(sim_duration / Ts);

    partC.time           = (0:steps-1)' * Ts;
    partC.voltage        = zeros(steps, 1);
    partC.yaw_rate       = zeros(steps, 1);
    partC.heading        = zeros(steps, 1);
    partC.waypoint       = NaN(steps, 2);
    partC.lateral_error  = zeros(steps, 1);
    partC.steering_angle = zeros(steps, 1);
    partC.heading_error  = zeros(steps, 1);

    % Initialize the simulator with explicit initial conditions/waypoints.
    [gps, yaw_gyro, counts, wp, lat_err] = run_Indy_car_Fall_25(0, Vel, X0, waypoint_file);
    clear run_Indy_car_Fall_25;

    acc_counts = 0;
    last_raw   = NaN;

    heading_int = 0;
    yaw_err_prev = 0;
    steer_int = 0;

    encoder_scale = (2*pi) / params.encoder.counts_per_rev;

    for k = 1:steps
        % Use previous count to estimate steering angle at start of step.
        if isnan(last_raw)
            acc_counts = counts;
        else
            delta = counts - last_raw;
            if delta > params.max_encoder / 2, delta = delta - params.max_encoder; end
            if delta < -params.max_encoder / 2, delta = delta + params.max_encoder; end
            acc_counts = acc_counts + delta;
        end
        last_raw = counts;

        steer_angle = (acc_counts * encoder_scale) / params.gear.N;

        desired_heading = atan2(wp(2) - gps(2), wp(1) - gps(1));
        head_err = wrap_to_pi(desired_heading - gps(3));
        heading_int = heading_int + head_err * Ts;

        yaw_cmd = controller.Kp3 * head_err + controller.Ki3 * heading_int;

        yaw_err = yaw_cmd - yaw_gyro;
        yaw_err_deriv = (yaw_err - yaw_err_prev) / Ts;
        yaw_err_prev = yaw_err;

        steer_cmd = controller.Kp2 * yaw_err + controller.Kd2 * yaw_err_deriv;
        steer_cmd = max(min(steer_cmd, steer_lim_rad), -steer_lim_rad);

        steer_err = steer_cmd - steer_angle;
        steer_int = steer_int + steer_err * Ts;

        voltage_cmd = controller.Kp1 * steer_err + controller.Ki1 * steer_int;
        voltage_cmd = max(min(voltage_cmd, voltage_limit), -voltage_limit);

        [gps, yaw_gyro, counts, wp, lat_err] = run_Indy_car_Fall_25(voltage_cmd, Vel);

        partC.voltage(k)        = voltage_cmd;
        partC.yaw_rate(k)       = yaw_gyro;
        partC.heading(k)        = gps(3);
        partC.waypoint(k, :)    = wp(:).';
        partC.lateral_error(k)  = lat_err;
        partC.steering_angle(k) = steer_angle;
        partC.heading_error(k)  = head_err;
    end
end

function val = get_option(opts, name, default)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = default;
    end
end

function angle = wrap_to_pi(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end