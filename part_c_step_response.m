function step_results = part_c_step_response(controller, params, opts)

    if nargin < 3
        opts = struct();
    end

    Ts            = get_option(opts, 'Ts', 0.001);
    sim_duration  = get_option(opts, 'sim_duration', 3);
    step_time     = get_option(opts, 'step_time', 0.001);
    step_deg      = get_option(opts, 'step_deg', 5);
    speeds        = get_option(opts, 'speeds', [8]);
    voltage_limit = get_option(opts, 'voltage_limit', 12);
    steer_lim_rad = deg2rad(get_option(opts, 'steering_limit_deg', 20));
    out_dir       = get_option(opts, 'output_dir', 'figures');
    yaw_tf        = get_option(opts, 'yaw_tf', []);
    X0            = get_option(opts, 'X0', [0 0 0 0 0]);

    out_dir_abs = out_dir;
    if ~isfolder(out_dir_abs)
        out_dir_abs = fullfile(pwd, out_dir);
        if ~isfolder(out_dir_abs)
            mkdir(out_dir_abs);
        end
    end

    yaw_model = [];
    if ~isempty(yaw_tf)
        yaw_model = c2d(ss(yaw_tf), Ts);
    end

    step_results = struct();
    for idx = 1:numel(speeds)
        Vtest = speeds(idx);
        result = run_heading_step(controller, params, Vtest, Ts, sim_duration, ...
                                  step_time, deg2rad(step_deg), voltage_limit, ...
                                  steer_lim_rad, X0, yaw_model);

        figure('Name', sprintf('Heading Step %.0f m/s', result.time(end) / result.time(2)));

        subplot(3,1,1);
        plot(result.time, result.heading_ref, 'k--', 'DisplayName', 'Reference'); hold on;
        plot(result.time, result.heading, 'b', 'DisplayName', 'Measured');
        if any(result.heading_linear)
            plot(result.time, result.heading_linear, 'Color', [0.85 0.33 0.1], ...
                 'DisplayName', 'Predicted');
        end
        grid on; ylabel('Heading [rad]');
        title('Heading step response'); legend('Location','best');
    
        subplot(3,1,2);
        plot(result.time, result.yaw_rate, 'b', 'DisplayName', 'Yaw rate'); hold on;
        if any(result.yaw_rate_linear)
            plot(result.time, result.yaw_rate_linear, 'Color', [0.85 0.33 0.1], ...
                 'DisplayName', 'Predicted yaw rate');
        end
        grid on; ylabel('Yaw rate [rad/s]'); legend('Location','best');
    
        subplot(3,1,3);
        plot(result.time, result.voltage, 'LineWidth', 1.2, 'DisplayName', 'Voltage'); hold on;
        plot(result.time, result.steer_cmd, '--', 'DisplayName', 'Steer cmd [rad]');
        plot(result.time, result.steering_angle, ':', 'DisplayName', 'Steer angle [rad]');
        grid on; xlabel('Time [s]'); ylabel('Control'); legend('Location','best');

        step_results.(sprintf('speed_%d', Vtest)) = result;
    end
end

function result = run_heading_step(controller, params, Vel, Ts, sim_duration, ...
                                   step_time, step_rad, voltage_limit, ...
                                   steer_lim_rad, X0, yaw_model)
    steps = floor(sim_duration / Ts);

    result.time           = (0:steps-1)' * Ts;
    result.heading_ref    = zeros(steps, 1);
    result.heading        = zeros(steps, 1);
    result.heading_linear = zeros(steps, 1);
    result.heading_error  = zeros(steps, 1);
    result.yaw_rate       = zeros(steps, 1);
    result.yaw_rate_linear= zeros(steps, 1);
    result.voltage        = zeros(steps, 1);
    result.steering_angle = zeros(steps, 1);
    result.steer_cmd      = zeros(steps, 1);
    result.yaw_cmd        = zeros(steps, 1);

    [gps, yaw_gyro, counts] = run_Indy_car_Fall_25(0, Vel, X0, 0);
    clear run_Indy_car_Fall_25;

    acc_counts = 0;
    last_raw   = NaN;

    heading_int = 0;
    yaw_err_prev = 0;
    steer_int = 0;

    encoder_scale = (2*pi) / params.encoder.counts_per_rev;

    x_lin = [];
    if ~isempty(yaw_model)
        x_lin = zeros(size(yaw_model.A, 1), 1);
    end

    for k = 1:steps
        t = result.time(k);
        heading_ref = 0;
        if t >= step_time
            heading_ref = step_rad;
        end
        result.heading_ref(k) = heading_ref;

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

        head_err = wrap_to_pi(heading_ref - gps(3));
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

        [gps, yaw_gyro, counts] = run_Indy_car_Fall_25(voltage_cmd, Vel);

        result.heading(k)        = gps(3);
        result.heading_error(k)  = head_err;
        result.yaw_rate(k)       = yaw_gyro;
        result.voltage(k)        = voltage_cmd;
        result.steering_angle(k) = steer_angle;
        result.steer_cmd(k)      = steer_cmd;
        result.yaw_cmd(k)        = yaw_cmd;

        if ~isempty(yaw_model)
            yaw_lin = yaw_model.C * x_lin + yaw_model.D * voltage_cmd;
            x_lin = yaw_model.A * x_lin + yaw_model.B * voltage_cmd;
            result.yaw_rate_linear(k) = yaw_lin;
            if k == 1
                result.heading_linear(k) = yaw_lin * Ts;
            else
                result.heading_linear(k) = result.heading_linear(k-1) + yaw_lin * Ts;
            end
        end
    end

    result.metrics = compute_step_metrics(result.time, result.heading, ...
                                          result.heading_ref, step_time);

    if ~isempty(yaw_model)
        result.metrics_linear = compute_step_metrics(result.time, ...
                                        result.heading_linear, ...
                                        result.heading_ref, step_time);
    end
end

function metrics = compute_step_metrics(time, response, reference, step_time)
    [~, step_idx] = min(abs(time - step_time));
    final_value = reference(end);
    tolerance = 0.05 * max(1e-6, abs(final_value));

    err = response - final_value;
    within = abs(err) <= tolerance;

    settle_idx = find(within(step_idx:end), 1, 'first');
    if isempty(settle_idx)
        settle_time = NaN;
    else
        idx_global = settle_idx + step_idx - 1;
        if all(within(idx_global:end))
            settle_time = time(idx_global) - step_time;
        else
            settle_time = NaN;
        end
    end

    overshoot_pct = (max(response(step_idx:end)) - final_value) / ...
                    max(1e-6, abs(final_value)) * 100;

    metrics = struct('final_value', final_value, ...
                     'tolerance', tolerance, ...
                     'settle_time', settle_time, ...
                     'overshoot_pct', overshoot_pct);
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