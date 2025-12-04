function varargout = SystemsfinalCollated(varargin)
%SYSTEMSFINALCOLLATED Unified access point for the SystemsFinal MATLAB codebase.
%   This single file aggregates the contents of all MATLAB functions in the
%   repository so the project can be shared or executed with one dependency.
%
%   The default call executes every assignment step end-to-end:
%     (A) Identify steering/rack dynamics and build the yaw-rate model.
%     (B) Design the cascaded steering/yaw/heading controllers with metrics.
%     (C) Run waypoint tracking on the provided p-code plant.
%     (D) Validate with heading step responses across requested speeds.
%     (E) Demonstrate IMS trajectory tracking (WP #2) with Google Earth export.
%
%   Usage:
%     results = SystemsfinalCollated(opts)       % Run Systemsfinalprod workflow.
%     [yaw_tf] = SystemsfinalCollated('build_bicycle_model', params, Vel)
%     ...and similarly for the other component functions listed below.
%
%   Supported function selectors (case-insensitive):
%     - 'systemsfinalprod' (default when first argument is not a char)
%     - 'build_bicycle_model'
%     - 'collect_pcode_response'
%     - 'controller_dev'
%     - 'pcode_identification'
%     - 'part_c_step_response'
%     - 'solve_part_c'
%     - 'steering'
%     - 'run_Indy_car_help' (returns help text string)
%
%   Example:
%       opts = struct('Vel', 10);
%       results = SystemsfinalCollated(opts);
%       yaw_tf = SystemsfinalCollated('build_bicycle_model', results.params, 10);
%
%   The implementation mirrors the original standalone .m files but keeps all
%   logic together for portability, including explicit outputs for steps A–E.

    if nargin == 0 || ~ischar(varargin{1})
        [varargout{1:nargout}] = Systemsfinalprod_local(varargin{:});
        return;
    end

    selector = lower(varargin{1});
    args = varargin(2:end);

    switch selector
        case {'systemsfinalprod', 'main'}
            [varargout{1:nargout}] = Systemsfinalprod_local(args{:});
        case 'build_bicycle_model'
            [varargout{1:nargout}] = build_bicycle_model_local(args{:});
        case 'collect_pcode_response'
            [varargout{1:nargout}] = collect_pcode_response_local(args{:});
        case 'controller_dev'
            [varargout{1:nargout}] = controller_dev_local(args{:});
        case 'pcode_identification'
            [varargout{1:nargout}] = pcode_identification_local(args{:});
        case 'part_c_step_response'
            [varargout{1:nargout}] = part_c_step_response_local(args{:});
        case 'solve_part_c'
            [varargout{1:nargout}] = solve_part_c_local(args{:});
        case 'steering'
            [varargout{1:nargout}] = steering_local(args{:});
        case {'run_indy_car_help', 'run_Indy_car_help'}
            [varargout{1:nargout}] = run_Indy_car_help_local();
        otherwise
            error('SystemsfinalCollated:UnknownFunction', ...
                  'Unknown function selector "%s".', varargin{1});
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% Collated main workflow %%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function results = Systemsfinalprod_local(user_options)
    clc; close all;

    % If the caller provided an options struct, use it. Otherwise start fresh.
    if nargin > 0
        opts = user_options;
    else
        opts = struct();
    end

    params = default_parameters_local();

    opts.Vel                = get_option_local(opts, 'Vel', 8);
    opts.Ts                 = get_option_local(opts, 'Ts', 0.001);
    opts.id_duration        = get_option_local(opts, 'id_duration', 3.5);
    opts.id_voltage         = get_option_local(opts, 'id_voltage', 1);
    opts.test_duration      = get_option_local(opts, 'test_duration', 3);
    opts.multi_sine_freqs   = get_option_local(opts, 'multi_sine_freqs', [0.5 1 2 5]);
    opts.multi_sine_amp     = get_option_local(opts, 'multi_sine_amp', 1);
    opts.controller_vel     = get_option_local(opts, 'controller_vel', 15);
    opts.show_partB_plots   = get_option_local(opts, 'show_partB_plots', true);
    opts.figure_prefix      = get_option_local(opts, 'figure_prefix', 'Part B - ');
    opts.partC_duration     = get_option_local(opts, 'partC_duration', 20);
    opts.partC_waypoint     = get_option_local(opts, 'partC_waypoint', 1);
    opts.voltage_limit      = get_option_local(opts, 'voltage_limit', 12);
    opts.steering_limit_deg = get_option_local(opts, 'steering_limit_deg', 20);
    % Evaluate heading steps at baseline (8 m/s) and high-speed (>60 m/s)
    opts.step_speeds        = get_option_local(opts, 'step_speeds', [8 60]);
    opts.step_time          = get_option_local(opts, 'step_time', 0.001);
    opts.step_deg           = get_option_local(opts, 'step_deg', 5);
    % IMS waypoint lap export for Google Earth / GPSVisualizer
    opts.partD_waypoint     = get_option_local(opts, 'partD_waypoint', 2);
    opts.partD_vel          = get_option_local(opts, 'partD_vel', 15);
    opts.partD_max_time     = get_option_local(opts, 'partD_max_time', 120);
    opts.partD_export_file  = get_option_local(opts, 'partD_export_file', ...
                                               'ims_track_run_for_google_earth.txt');

    if isfield(opts, 'params')
        params = merge_structs_local(params, opts.params);
    end

    results = struct();
    results.opts = opts;
    results.params = params;

    %% Identification: collect step response from p-code to solve for Je and Be
    clear pcode_identification;
    id_time = (0:opts.Ts:opts.id_duration)'; % Time vector for identification
    const_volts = zeros(size(id_time));
    const_volts(:) = opts.id_voltage;
    SS_values = pcode_identification_local(const_volts, id_time, opts, params);
    results.SS_values = SS_values;

    %% Build the yaw-rate bicycle model and cascade with rack model
    const_rack_model = steering_local(params, SS_values);
    [yaw_tf] = build_bicycle_model_local(params, opts.Vel);
    const_voltage_to_yaw_model = series(const_rack_model, yaw_tf);
    results.const_voltage_to_yaw_model = const_voltage_to_yaw_model;

    figure('Name','Bode');
    bode(const_voltage_to_yaw_model);
    figure('Name','Eigenvalues');
    p = pole(const_voltage_to_yaw_model);
    plot(p, 0 , 'X', 'MarkerSize',10, 'LineWidth',3);
    xlabel('Real'); ylabel('Imag');
    title('Poles / Eigenvalues'); grid on;

    figure('Name', 'Yaw Rate Comparison Constant Input');
    plot(id_time, SS_values.yaw_rate, 'r', 'DisplayName', 'Yaw Rate'); hold on;
    step(const_voltage_to_yaw_model, opts.id_duration);
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate [Rad/s]');
    legend('show'); title('Yaw Rate Comparison');

    %% Bode verification with multiple sinusoidal inputs
    multi_sine_time = (0:opts.Ts:opts.test_duration)';
    multi_sine_input = zeros(numel(multi_sine_time), 1);
    for f = opts.multi_sine_freqs
        multi_sine_input = multi_sine_input + opts.multi_sine_amp * sin(2*pi*f*multi_sine_time);
    end
    multi_sine_data = collect_pcode_response_local(multi_sine_input, multi_sine_time, opts, params, SS_values, const_voltage_to_yaw_model);
    multi_sine_tf = lsim(const_voltage_to_yaw_model, multi_sine_input, multi_sine_time);

    figure('Name', 'Multi-Sine Frequency Verification');
    subplot(3,1,1);
    plot(multi_sine_time, multi_sine_input, 'k', 'DisplayName', 'Input Voltage');
    grid on; ylabel('Volts [V]'); legend('show'); title('Composite Sine Excitation');

    subplot(3,1,2);
    plot(multi_sine_time, multi_sine_data.yaw_rate, 'r', 'DisplayName', 'P-code'); hold on;
    plot(multi_sine_time, multi_sine_tf, 'b', 'DisplayName', 'Transfer Function');
    grid on; ylabel('Yaw Rate [rad/s]'); legend('show'); title('Yaw Response to Multi-Sine Input');

    subplot(3,1,3);
    measured_amp = arrayfun(@(f) steady_state_amplitude_local(multi_sine_data.yaw_rate, multi_sine_time, f), opts.multi_sine_freqs);
    predicted_amp = arrayfun(@(f) steady_state_amplitude_local(multi_sine_tf, multi_sine_time, f), opts.multi_sine_freqs);
    stem(opts.multi_sine_freqs, measured_amp, 'filled', 'DisplayName', 'Measured'); hold on;
    stem(opts.multi_sine_freqs, predicted_amp, 'LineStyle', '--', 'DisplayName', 'Transfer Function');
    grid on; xlabel('Frequency [Hz]'); ylabel('Amplitude [rad/s]');
    legend('show'); title('Frequency Response Amplitude Check');

    %% Test the transfer function against a varying voltage profile
    clear collect_pcode_response;
    id_time = (0:opts.Ts:opts.test_duration)';
    varr_volts = 0.8*sin(2*pi*0.5*id_time) + 0.4*sin(2*pi*2*id_time);
    varr_data = collect_pcode_response_local(varr_volts, id_time, opts, params, SS_values, const_voltage_to_yaw_model);

    figure('Name', 'Yaw Rate Comparison Variable Input');
    subplot(2,1,1)
    plot(id_time,varr_volts, 'b', 'DisplayName', 'Input Voltage');
    grid on; xlabel('Time [s]'); ylabel('Volts [V]'); title('Varying Voltage Profile');
    subplot(2,1,2)
    plot(id_time, varr_data.yaw_rate, 'r', 'DisplayName', 'P-code'); hold on;
    plot(id_time, varr_data.sim_yaw, 'b', 'DisplayName', 'Transfer Function');
    grid on; xlabel('Time [s]'); ylabel('Yaw Rate [Rad/s]');
    legend('show'); title('Yaw Rate Comparison');

    results.identification.id_time = id_time;
    results.identification.const_input = const_volts;
    results.identification.multi_sine_input = multi_sine_input;
    results.identification.multi_sine_time = multi_sine_time;
    results.identification.varr_volts = varr_volts;
    results.identification.varr_data = varr_data;

    results.stepA = struct('SS_values', SS_values, ...
                           'const_voltage_to_yaw_model', const_voltage_to_yaw_model, ...
                           'multi_sine_data', multi_sine_data, ...
                           'multi_sine_tf', multi_sine_tf, ...
                           'varr_data', varr_data);

    %% Part B: controller synthesis + verification
    plot_config.show_plots = opts.show_partB_plots;
    plot_config.figure_prefix = opts.figure_prefix;

    controls = controller_dev_local(params, opts.controller_vel, SS_values, plot_config);
    results.controller = controls;
    results.stepB = controls;

    %% Part C: waypoint tracking with cascaded controller
    partC_opts = struct('Ts', opts.Ts, ...
                        'sim_duration', opts.partC_duration, ...
                        'Vel', opts.controller_vel, ...
                        'waypoint_file', sanitize_waypoint_file_local(opts.partC_waypoint), ...
                        'voltage_limit', opts.voltage_limit, ...
                        'steering_limit_deg', opts.steering_limit_deg);

    results.partC = solve_part_c_local(controls, params, partC_opts);
    results.stepC = results.partC;

    %% Part C validation: heading step response on the p-code plant
    step_opts = struct('Ts', opts.Ts, ...
                      'sim_duration', 3, ...
                      'step_time', opts.step_time, ...
                      'step_deg', opts.step_deg, ...
                      'speeds', opts.step_speeds, ...
                      'voltage_limit', opts.voltage_limit, ...
                      'steering_limit_deg', opts.steering_limit_deg, ...
                      'yaw_tf', const_voltage_to_yaw_model);

    results.step_response = part_c_step_response_local(controls, params, step_opts);
    results.stepD = results.step_response;

    %% Part D: IMS trajectory (WP #2) at 15 m/s with Google Earth export
    traj_opts = struct('Ts', opts.Ts, ...
                       'Vel', opts.partD_vel, ...
                       'waypoint_file', sanitize_waypoint_file_local(opts.partD_waypoint), ...
                       'voltage_limit', opts.voltage_limit, ...
                       'steering_limit_deg', opts.steering_limit_deg, ...
                       'max_time', opts.partD_max_time, ...
                       'export_file', opts.partD_export_file);

    results.partD = run_trajectory_validation_local(controls, params, traj_opts);
    results.stepE = results.partD;
end

%%%%%%%%%%%%%%%%%%%
%%% Local helpers
%%%%%%%%%%%%%%%%%%%

function params = default_parameters_local()
    params = struct();
    params.encoder.counts_per_rev = 500 * 4;
    params.max_encoder  = 4096;
    params.motor.R      = 0.9;
    params.motor.L      = 0.45e-3;
    params.gear.N       = 21*15;
    params.vehicle.mass = 720;
    params.vehicle.Iz   = 1200;
    params.vehicle.a    = 1.72;
    params.vehicle.b    = 1.25;
    params.vehicle.Cf   = 100000;
    params.vehicle.Cr   = 100000;
    params.vehicle.Kt   = 0.0259;
end

function wp_file = sanitize_waypoint_file_local(wp_file)
    % Ensure the waypoint selection matches the p-code availability.
    % Valid files are 0 (none), 1 (double lane change), 2 (IMS), 3 (Barber).
    % File 4 (NCAT) is explicitly unavailable in the provided p-code.

    if isempty(wp_file) || ~isnumeric(wp_file) || ~isscalar(wp_file)
        warning('Waypoint selector must be a numeric scalar. Falling back to IMS (2).');
        wp_file = 2;
        return;
    end

    if wp_file == 4
        warning('Waypoint file 4 (NCAT) is not available in the supplied p-code. Using IMS (2) instead.');
        wp_file = 2;
    elseif wp_file < 0 || wp_file > 3
        warning('Waypoint selector %g is out of range (0-3). Using IMS (2) instead.', wp_file);
        wp_file = 2;
    end
end

function [gps, yaw_gyro, counts, wp, lat_err, wp_file_used] = kickoff_run_indy_with_fallback_local(voltage, Vel, X0, wp_file)
    % Start the p-code simulator with a safe waypoint selection. If the p-code
    % throws the known "WP_array" error (e.g., NCAT unavailable), automatically
    % revert to IMS (2) so the workflow can proceed.

    wp_file_used = sanitize_waypoint_file_local(wp_file);

    try
        [gps, yaw_gyro, counts, wp, lat_err] = run_Indy_car_Fall_25(voltage, Vel, X0, wp_file_used);
    catch ME
        if contains(ME.message, 'WP_array')
            fallback_wp = 2;
            warning('Waypoint file %d failed to load (likely unavailable). Falling back to IMS (%d).', ...
                    wp_file_used, fallback_wp);
            wp_file_used = fallback_wp;
            [gps, yaw_gyro, counts, wp, lat_err] = run_Indy_car_Fall_25(voltage, Vel, X0, wp_file_used);
        else
            rethrow(ME);
        end
    end
end

function amp = steady_state_amplitude_local(signal, time_vector, freq_hz)
    % Estimate steady-state amplitude of a sinusoidal component by focusing on
    % the latter half of the record, fitting to a sine at freq_hz, and
    % returning the peak magnitude.
    tail_start = ceil(numel(signal) / 2);
    t_tail = time_vector(tail_start:end);
    y_tail = signal(tail_start:end);

    basis = [sin(2*pi*freq_hz*t_tail), cos(2*pi*freq_hz*t_tail)];
    coeffs = basis \ y_tail;
    amp = sqrt(sum(coeffs.^2));
end

function merged = merge_structs_local(base, override)
    merged = base;
    names = fieldnames(override);
    for k = 1:numel(names)
        f = names{k};
        if isstruct(override.(f))
            merged.(f) = merge_structs_local(base.(f), override.(f));
        else
            merged.(f) = override.(f);
        end
    end
end

function val = get_option_local(opts, name, default)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = default;
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%
%%% Collated functions
%%%%%%%%%%%%%%%%%%%%%%%%

function [yaw_tf] = build_bicycle_model_local(params, Vel)
    m  = params.vehicle.mass;
    Iz = params.vehicle.Iz;
    a  = params.vehicle.a;
    b  = params.vehicle.b;
    Cf = params.vehicle.Cf;
    Cr = params.vehicle.Cr;
    s = tf('s');

    num = (a*Cf)*s + (Cf*Cr*(a+b))/(m*Vel);
    den = (Iz)*s^2 + (((Cf+Cr)*Iz + m*((a^2)*Cf+(b^2)*Cr))/(m*Vel))*s + ((Cf+Cr)*((a^2)*Cf+(b^2)*Cr) - ((a*Cf-b*Cr)*m*Vel^2) - (a*Cf - b*Cr)^2)/(m*(Vel^2));
    yaw_tf = num/den;
end

function data = collect_pcode_response_local(voltage, time, opts, params, SS_values, yaw_model)
    steps = numel(time);
    data.time     = time(:);
    data.voltage  = voltage(:);
    data.yaw_rate = zeros(steps,1);
    data.motor_angle_counts = zeros(steps,1);
    data.sim_yaw = zeros(steps,1);

    acc_counts   = 0;
    last_raw     = NaN;
    theta_counts = zeros(steps, 1);
    if nargin < 6 || isempty(yaw_model)
        const_rack_model = steering_local(params, SS_values);
        yaw_tf = build_bicycle_model_local(params, opts.Vel);
        yaw_model = series(const_rack_model, yaw_tf);
    end

    [~, ~, ~] = run_Indy_car_Fall_25(0,0,[0 0 0 0 0],0);
    data.sim_yaw(1) = 0;

    %Gather

    clear run_Indy_car_Fall_25;

    for k = 1:steps
        [~, gyro_k, counts] = run_Indy_car_Fall_25(voltage(k), opts.Vel);
        data.yaw_rate(k)          = (gyro_k);

        if isnan(last_raw)
            acc_counts = counts;
        else
            delta = counts - last_raw;
            if delta >  params.max_encoder / 2, delta = delta - params.max_encoder; end
            if delta < -params.max_encoder / 2, delta = delta + params.max_encoder; end
            acc_counts = acc_counts + delta;
        end
        last_raw = counts;
        theta_counts(k) = acc_counts;
    end

   %motor counts
   theta_m = (theta_counts * (2*pi / params.encoder.counts_per_rev));
   %motor to steering
   steering_degree = (theta_m / params.gear.N);
   data.motor_angle = steering_degree;

   % Transfer function response to the same voltage profile
   data.sim_yaw = lsim(yaw_model, voltage(:), time(:));
end

function controller = controller_dev_local(params, velocity, SS_values, plot_opts)

    if nargin < 4
        plot_opts = struct();
    end
    if ~isfield(plot_opts, 'show_plots')
        plot_opts.show_plots = true;
    end
    if ~isfield(plot_opts, 'figure_prefix')
        plot_opts.figure_prefix = 'Part B - ';
    end

    % Vehicle parameters
    m  = params.vehicle.mass;
    Iz = params.vehicle.Iz;
    a  = params.vehicle.a;
    b  = params.vehicle.b;
    Cf = params.vehicle.Cf;
    Cr = params.vehicle.Cr;

    % Rack parameters from identification
    K   = SS_values.K;
    Je  = SS_values.Je;
    Be  = SS_values.Be;
    tau = Je / Be;

    % Desired dynamics
    zeta = 0.8;   % damping ratio
    TTS  = 0.5;   % target time-to-settle [s]

    % Bicycle model shorthand
    C0 = Cf + Cr;
    C1 = a*Cf + b*Cr;
    C2 = (a^2)*Cf + (b^2)*Cr;

    A = a*Cf / Iz;
    B = (a+b)*Cr*Cf / (Iz*m*velocity);
    C = (C0*Iz + C2*m)/(Iz*m*velocity);
    D = ((C0*C2 - C1*m*velocity^2) - C1^2)/(Iz*m*velocity^2);

    % Inner steering loop (PI)
    omega_n_delta = 4 / (zeta*TTS);
    controller.Kp1 = (2*tau*omega_n_delta - 1) / K;
    controller.Ki1 = (tau*omega_n_delta^2) / K;

    % Yaw-rate loop (PD)
    omega_n_r = 4 / (zeta*(TTS));
    controller.Kp2 = -((-A*C*(omega_n_r^2) + 2*A*D*omega_n_r*zeta - B*D + B*(omega_n_r^2)) / ((A^2)*(omega_n_r^2) - 2*A*B*omega_n_r*zeta + B^2));
    controller.Kd2 = -((A*D - A*omega_n_r^2 - B*C + 2*B*omega_n_r*zeta) / ((A^2)*(omega_n_r^2) - 2*A*B*omega_n_r*zeta + B^2));

    % Heading loop (PI)
    omega_n_psi = 4 / (zeta*TTS);
    controller.Kp3 = omega_n_psi*2*zeta;
    controller.Ki3 = omega_n_psi^2;

    s = tf('s');

    % Plants
    G_delta  = K/(tau*s + 1);              % V -> steering angle
    G_rdelta = (A*s + B)/(s^2 + C*s + D);  % steering -> yaw rate

    % Controllers
    C_delta = controller.Kp1 + controller.Ki1/s;    % inner PI (δ-loop)
    C_r     = controller.Kp2 + controller.Kd2*s;    % yaw-rate PD
    C_psi   = controller.Kp3 + controller.Ki3/s;    % outer PI (heading)

    %% Closed-loop interconnections
    % 1) Inner steering loop: δ_ref -> δ
    T_delta = feedback(C_delta*G_delta, 1);

    % 2) Yaw-rate loop: r_ref -> r (uses closed δ-loop as actuator)
    L_r = C_r * G_rdelta * T_delta;
    T_r = feedback(L_r, 1);

    % 3) Heading loop: ψ_ref -> ψ
    G_psi_eff = T_r / s;             % r_ref -> ψ is yaw-loop then integrator
    L_psi = C_psi * G_psi_eff;
    T_psi = feedback(L_psi, 1);

    %% Test metrics (Part B requirements)
    controller.loops.delta.tf = minreal(T_delta);
    controller.loops.delta.poles = pole(T_delta);
    controller.loops.delta.step = stepinfo(T_delta);
    controller.loops.delta.dc_gain = dcgain(T_delta);

    controller.loops.r.tf = minreal(T_r);
    controller.loops.r.poles = pole(T_r);
    controller.loops.r.step = stepinfo(T_r);
    controller.loops.r.bandwidth = bandwidth(T_r);

    controller.loops.psi.tf = minreal(T_psi);
    controller.loops.psi.poles = pole(T_psi);
    controller.loops.psi.step = stepinfo(T_psi);
    controller.loops.psi.dc_gain = dcgain(T_psi);

    controller.TF = controller.loops.psi.tf;

    %% Low-frequency tracking evaluation (psi_ref = sin(1*t))
    sine_eval.freq_rad_s = 1;                 % rad/s command frequency
    sine_eval.amp_rad    = deg2rad(5);        % reasonable 5 deg heading request
    sine_eval.time       = (0:0.01:25)';      % long enough for steady-state
    sine_eval.psi_ref    = sine_eval.amp_rad * sin(sine_eval.freq_rad_s * sine_eval.time);
    [sine_eval.psi_resp, sine_eval.time] = lsim(T_psi, sine_eval.psi_ref, sine_eval.time);

    [ref_amp, ref_phase]   = fit_sine_local(sine_eval.psi_ref, sine_eval.time, sine_eval.freq_rad_s);
    [resp_amp, resp_phase] = fit_sine_local(sine_eval.psi_resp, sine_eval.time, sine_eval.freq_rad_s);
    sine_eval.amp_ratio    = resp_amp / ref_amp;
    sine_eval.phase_lag_deg = wrap_to_180_local(rad2deg(resp_phase - ref_phase));

    [bode_mag, bode_phase, ~] = bode(T_psi, sine_eval.freq_rad_s);
    sine_eval.bode_mag        = squeeze(bode_mag);
    sine_eval.bode_phase_deg  = squeeze(bode_phase);

    controller.sine_eval = sine_eval;

    if plot_opts.show_plots
        prefix = plot_opts.figure_prefix;
        quick_step_plot_local(T_delta, [prefix 'Inner steering: \delta_{ref} -> \delta']);
        quick_step_plot_local(T_r, [prefix 'Yaw-rate loop: r_{ref} -> r']);
        quick_step_plot_local(T_psi, [prefix 'Heading loop: \psi_{ref} -> \psi']);

        pole_plot_local(controller.loops.delta.poles, [prefix 'Steering loop poles']);
        pole_plot_local(controller.loops.r.poles, [prefix 'Yaw-rate loop poles']);
        pole_plot_local(controller.loops.psi.poles, [prefix 'Heading loop poles']);

        figure('Name', [prefix 'Heading loop Bode']);
        bode(T_psi);
        grid on;

        figure('Name', [prefix 'Heading sinusoid tracking']);
        plot(sine_eval.time, sine_eval.psi_ref, 'k--', 'DisplayName', '\psi_{ref} (5 deg)'); hold on;
        plot(sine_eval.time, sine_eval.psi_resp, 'b', 'DisplayName', '\psi response');
        grid on; xlabel('Time [s]'); ylabel('Heading [rad]');
        title([prefix 'Tracking of \psi_{ref} = 5 deg \times sin(1t)']);
        legend('show');
        text(0.05*max(sine_eval.time), 0.8*max(sine_eval.amp_rad), ...
            sprintf('Amp ratio: %.3f, Phase lag: %.1f deg', ...
            sine_eval.amp_ratio, sine_eval.phase_lag_deg));
    end
end

%%%%%%%%%%%%%%%%%%%
%%% Local helpers
%%%%%%%%%%%%%%%%%%%
function quick_step_plot_local(T, title_str)
    figure('Name', title_str);
    step(T);
    grid on;
    title(title_str);
end

function pole_plot_local(poles, title_str)
    figure('Name', title_str);
    plot(real(poles), imag(poles), 'x', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('Real'); ylabel('Imag'); grid on; title(title_str);
end

function [amp, phase] = fit_sine_local(signal, time_vector, freq_rad_s)
    % Estimate amplitude and phase of a sinusoid at freq_rad_s using the
    % tail of the record to ignore transients.
    tail_start = ceil(numel(signal) / 2);
    t_tail = time_vector(tail_start:end);
    y_tail = signal(tail_start:end);

    basis = [sin(freq_rad_s * t_tail), cos(freq_rad_s * t_tail)];
    coeffs = basis \ y_tail;

    amp = sqrt(sum(coeffs.^2));
    phase = atan2(coeffs(2), coeffs(1));
end

function deg_wrapped = wrap_to_180_local(degrees)
    % Wrap angles to the [-180, 180] deg range for reporting phase lags.
    deg_wrapped = mod(degrees + 180, 360) - 180;
end

function SS_values = pcode_identification_local(voltage, time, opts, params)
    steps = numel(time);
    acc_counts   = 0;
    last_raw     = NaN;
    theta_counts = zeros(steps, 1);

    [~, ~, ~] = run_Indy_car_Fall_25(0,0,[0 0 0 0 0],0);

    %Gather

    clear run_Indy_car_Fall_25;

    for k = 1:steps
        [~, gyro_k, counts] = run_Indy_car_Fall_25(voltage(k), opts.Vel);
        SS_values.yaw_rate(k)          = (gyro_k);

        if isnan(last_raw)
            acc_counts = counts;
        else
            delta = counts - last_raw;
            if delta >  params.max_encoder / 2, delta = delta - params.max_encoder; end
            if delta < -params.max_encoder / 2, delta = delta + params.max_encoder; end
            acc_counts = acc_counts + delta;
        end
        last_raw = counts;
        theta_counts(k) = acc_counts;
    end

   %motor counts
   theta_m = (theta_counts * (2*pi / params.encoder.counts_per_rev));
   %motor to steering
   steering_degree = (theta_m / params.gear.N);

    derivative = gradient(steering_degree, time);
    SS_values.Be_multiplier = max(derivative);
    idx = find(derivative > (SS_values.Be_multiplier * .632));
    SS_values.tau = idx(1) * opts.Ts;
    SS_values.Be= params.gear.N*params.vehicle.Kt/SS_values.Be_multiplier;
    SS_values.Je = SS_values.tau*SS_values.Be;
    SS_values.K = params.gear.N * params.vehicle.Kt/ SS_values.Be;


    % Plot the steering degree over time
    s = tf('s');
    figure('Name', 'System Identification');
    subplot(2,1,1)
    plot(time, steering_degree, 'DisplayName', 'P-code');hold on;
    grid on;hold on;
    [y,t] = step((params.gear.N*params.vehicle.Kt)/(SS_values.Je*s^2 + SS_values.Be*s), time);
    plot(t,y, 'r', 'DisplayName', 'Simulated');
    xlabel('Time (s)');
    ylabel('Steering Degree (rads)');
    title('Steering Degree vs Time');
    legend('show')
    % Calculate the derivative of the steering degree

    % Plot the derivative of the steering degree over time
    subplot(2,1,2)
    plot(time, derivative,'DisplayName', 'P-code');
    grid on; hold on
    [y,t] = step((params.gear.N*params.vehicle.Kt)/(SS_values.Je*s + SS_values.Be), time);
    plot(t,y, 'r', 'DisplayName', 'Simulated');
    xlabel('Time (s)');
    ylabel('Rate of Change of Steering Degree (rads/s)');
    title('Rate of Change of Steering Degree vs Time');
    legend('show')
end

function step_results = part_c_step_response_local(controller, params, opts)

    if nargin < 3
        opts = struct();
    end

    Ts            = get_option_local(opts, 'Ts', 0.001);
    sim_duration  = get_option_local(opts, 'sim_duration', 3);
    step_time     = get_option_local(opts, 'step_time', 0.001);
    step_deg      = get_option_local(opts, 'step_deg', 5);
    speeds        = get_option_local(opts, 'speeds', [8]);
    voltage_limit = get_option_local(opts, 'voltage_limit', 12);
    steer_lim_rad = deg2rad(get_option_local(opts, 'steering_limit_deg', 20));
    out_dir       = get_option_local(opts, 'output_dir', 'figures');
    yaw_tf        = get_option_local(opts, 'yaw_tf', []);
    X0            = get_option_local(opts, 'X0', [0 0 0 0 0]);

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
    settle_target = 0.6;   % seconds
    overshoot_target = 5;  % percent
    for idx = 1:numel(speeds)
        Vtest = speeds(idx);
        result = run_heading_step_local(controller, params, Vtest, Ts, sim_duration, ...
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
        step_results.summary(idx) = struct( ...
            'speed_mps', Vtest, ...
            'settle_time_s', result.metrics.settle_time, ...
            'overshoot_pct', result.metrics.overshoot_pct, ...
            'meets_settle', result.metrics.settle_time <= settle_target, ...
            'meets_overshoot', result.metrics.overshoot_pct <= overshoot_target);
    end
end

function result = run_heading_step_local(controller, params, Vel, Ts, sim_duration, ...
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

        head_err = wrap_to_pi_local(heading_ref - gps(3));
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

    result.metrics = compute_step_metrics_local(result.time, result.heading, ...
                                          result.heading_ref, step_time);

    if ~isempty(yaw_model)
        result.metrics_linear = compute_step_metrics_local(result.time, ...
                                        result.heading_linear, ...
                                        result.heading_ref, step_time);
    end
end

function metrics = compute_step_metrics_local(time, response, reference, step_time)
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

function angle = wrap_to_pi_local(angle)
    angle = mod(angle + pi, 2*pi) - pi;
end

function out = ternary_local(condition, val_true, val_false)
    if condition
        out = val_true;
    else
        out = val_false;
    end
end

function partC = solve_part_c_local(controller, params, opts, X0)

    if nargin < 3
        opts = struct();
    end
    if nargin < 4
        X0 = [0 0 0 0 0];
    end

    Ts            = get_option_local(opts, 'Ts', 0.001);
    sim_duration  = get_option_local(opts, 'sim_duration', 20);
    Vel           = get_option_local(opts, 'Vel', 8);
    waypoint_file = get_option_local(opts, 'waypoint_file', 1);
    voltage_limit = get_option_local(opts, 'voltage_limit', 12);
    steer_lim_rad = deg2rad(get_option_local(opts, 'steering_limit_deg', 20));

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
    [gps, yaw_gyro, counts, wp, lat_err, waypoint_file] = ...
        kickoff_run_indy_with_fallback_local(0, Vel, X0, waypoint_file);
    partC.used_waypoint_file = waypoint_file;
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
        head_err = wrap_to_pi_local(desired_heading - gps(3));
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

function traj = run_trajectory_validation_local(controller, params, opts, X0)
    % Execute a full IMS (WP #2) lap at 15 m/s and export the GPS track for
    % Google Earth / GPSVisualizer review.

    if nargin < 3
        opts = struct();
    end
    if nargin < 4
        X0 = [0 0 pi 0 0];
    end

    Ts             = get_option_local(opts, 'Ts', 0.001);
    Vel            = get_option_local(opts, 'Vel', 15);
    waypoint_file  = get_option_local(opts, 'waypoint_file', 2);
    voltage_limit  = get_option_local(opts, 'voltage_limit', 12);
    steer_lim_rad  = deg2rad(get_option_local(opts, 'steering_limit_deg', 20));
    max_time       = get_option_local(opts, 'max_time', 120);
    export_file    = get_option_local(opts, 'export_file', 'ims_track_run_for_google_earth.txt');

    steps = floor(max_time / Ts);

    traj.time           = (0:steps-1)' * Ts;
    traj.voltage        = zeros(steps, 1);
    traj.yaw_rate       = zeros(steps, 1);
    traj.heading        = zeros(steps, 1);
    traj.heading_error  = zeros(steps, 1);
    traj.steering_angle = zeros(steps, 1);
    traj.steer_cmd      = zeros(steps, 1);
    traj.yaw_cmd        = zeros(steps, 1);
    traj.waypoint       = NaN(steps, 2);
    traj.lateral_error  = zeros(steps, 1);
    traj.position_EN    = zeros(steps, 2);

    % Kick off the simulator; clear after to avoid repeated clears inside loop.
    [gps, yaw_gyro, counts, wp, lat_err, waypoint_file] = ...
        kickoff_run_indy_with_fallback_local(0, Vel, X0, waypoint_file);
    traj.used_waypoint_file = waypoint_file;
    clear run_Indy_car_Fall_25;

    acc_counts = 0;
    last_raw   = NaN;

    heading_int = 0;
    yaw_err_prev = 0;
    steer_int = 0;

    encoder_scale = (2*pi) / params.encoder.counts_per_rev;

    completed = false;
    last_idx = steps;

    for k = 1:steps
        if any(isnan(wp))
            completed = true;
            last_idx = k - 1;
            break;
        end

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
        head_err = wrap_to_pi_local(desired_heading - gps(3));
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

        traj.voltage(k)        = voltage_cmd;
        traj.yaw_rate(k)       = yaw_gyro;
        traj.heading(k)        = gps(3);
        traj.heading_error(k)  = head_err;
        traj.steering_angle(k) = steer_angle;
        traj.steer_cmd(k)      = steer_cmd;
        traj.yaw_cmd(k)        = yaw_cmd;
        traj.waypoint(k, :)    = wp(:).';
        traj.lateral_error(k)  = lat_err;
        traj.position_EN(k, :) = gps(1:2);
    end

    if last_idx < 1
        last_idx = 1;
    end

    traj.time = traj.time(1:last_idx);
    traj.voltage = traj.voltage(1:last_idx);
    traj.yaw_rate = traj.yaw_rate(1:last_idx);
    traj.heading = traj.heading(1:last_idx);
    traj.heading_error = traj.heading_error(1:last_idx);
    traj.steering_angle = traj.steering_angle(1:last_idx);
    traj.steer_cmd = traj.steer_cmd(1:last_idx);
    traj.yaw_cmd = traj.yaw_cmd(1:last_idx);
    traj.waypoint = traj.waypoint(1:last_idx, :);
    traj.lateral_error = traj.lateral_error(1:last_idx);
    traj.position_EN = traj.position_EN(1:last_idx, :);

    traj.metrics = struct();
    traj.metrics.completed_lap = completed;
    traj.metrics.elapsed_time = traj.time(end);
    traj.metrics.exit_reason = ternary_local(completed, 'waypoint list exhausted (lap complete)', ...
                                             'time limit reached before lap completion');
    traj.metrics.mean_abs_lat_error = mean(abs(traj.lateral_error(~isnan(traj.lateral_error))));
    traj.metrics.max_abs_lat_error = max(abs(traj.lateral_error(~isnan(traj.lateral_error))));
    traj.metrics.waypoint_file_used = waypoint_file;

    figure('Name', 'Part D - IMS trajectory plan view');
    plot(traj.position_EN(:,1), traj.position_EN(:,2), 'b', 'DisplayName', 'Vehicle path'); hold on;
    plot(traj.waypoint(:,1), traj.waypoint(:,2), 'r--', 'DisplayName', 'Waypoints');
    grid on; axis equal;
    xlabel('East [m]'); ylabel('North [m]');
    title('IMS waypoint tracking at 15 m/s');
    legend('show');

    % Preserve the GPSVisualizer export created by the simulator so users can
    % upload to Google Earth without it being overwritten by subsequent runs.
    default_export = fullfile(fileparts(mfilename('fullpath')), 'waypoint_file_for_GPSVisualizer.txt');
    if exist(default_export, 'file') == 2 && ~isempty(export_file)
        copyfile(default_export, export_file);
        traj.export_file = export_file;
    else
        traj.export_file = '';
    end
end

function G = steering_local(params, coefficients)
    s = tf('s');
    N = params.gear.N;
    Kt = params.vehicle.Kt;
    Je = coefficients.Je;
    Be = coefficients.Be;

    G = (N*Kt)/(Je*s^2 + Be*s);
end

function help_text = run_Indy_car_help_local()
%RUN_INDY_CAR_HELP_LOCAL Provide the documentation string from run_Indy_car_help (2).m.
%   Returns the long-form usage description originally distributed alongside
%   the simulator p-code.
    help_text = fileread(fullfile(fileparts(mfilename('fullpath')), 'run_Indy_car_help (2).m'));
end
