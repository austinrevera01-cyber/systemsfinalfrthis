function results = Systemsfinalprod(user_options)

clc; close all;

% If the caller provided an options struct, use it. Otherwise start fresh.
if nargin > 0
    opts = user_options;
else
    opts = struct();
end

function params = default_parameters()
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

opts.Vel                = get_option(opts, 'Vel', 8);
opts.Ts                 = get_option(opts, 'Ts', 0.001);
opts.id_duration        = get_option(opts, 'id_duration', 3.5);
opts.id_voltage         = get_option(opts, 'id_voltage', 1);
opts.test_duration      = get_option(opts, 'test_duration', 3);
opts.multi_sine_freqs   = get_option(opts, 'multi_sine_freqs', [0.5 1 2 5]);
opts.multi_sine_amp     = get_option(opts, 'multi_sine_amp', 1);
opts.controller_vel     = get_option(opts, 'controller_vel', 15);
opts.show_partB_plots   = get_option(opts, 'show_partB_plots', true);
opts.figure_prefix      = get_option(opts, 'figure_prefix', 'Part B - ');
opts.partC_duration     = get_option(opts, 'partC_duration', 20);
opts.partC_waypoint     = get_option(opts, 'partC_waypoint', 1);
opts.voltage_limit      = get_option(opts, 'voltage_limit', 12);
opts.steering_limit_deg = get_option(opts, 'steering_limit_deg', 20);
opts.step_speeds        = get_option(opts, 'step_speeds', [8]);
opts.step_time          = get_option(opts, 'step_time', 0.001);
opts.step_deg           = get_option(opts, 'step_deg', 5);

params = default_parameters();
if isfield(opts, 'params')
    params = merge_structs(params, opts.params);
end

results = struct();
results.opts = opts;
results.params = params;

%% Identification: collect step response from p-code to solve for Je and Be
clear pcode_identification;
id_time = (0:opts.Ts:opts.id_duration)'; % Time vector for identification
const_volts = zeros(size(id_time));
const_volts(:) = opts.id_voltage;
SS_values = pcode_identification(const_volts, id_time, opts, params);
results.SS_values = SS_values;

%% Build the yaw-rate bicycle model and cascade with rack model
const_rack_model = steering(params, SS_values);
[yaw_tf] = build_bicycle_model(params, opts.Vel);
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
multi_sine_data = collect_pcode_response(multi_sine_input, multi_sine_time, opts, params, SS_values, const_voltage_to_yaw_model);
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
measured_amp = arrayfun(@(f) steady_state_amplitude(multi_sine_data.yaw_rate, multi_sine_time, f), opts.multi_sine_freqs);
predicted_amp = arrayfun(@(f) steady_state_amplitude(multi_sine_tf, multi_sine_time, f), opts.multi_sine_freqs);
stem(opts.multi_sine_freqs, measured_amp, 'filled', 'DisplayName', 'Measured'); hold on;
stem(opts.multi_sine_freqs, predicted_amp, 'LineStyle', '--', 'DisplayName', 'Transfer Function');
grid on; xlabel('Frequency [Hz]'); ylabel('Amplitude [rad/s]');
legend('show'); title('Frequency Response Amplitude Check');

%% Test the transfer function against a varying voltage profile
clear collect_pcode_response;
id_time = (0:opts.Ts:opts.test_duration)';
varr_volts = 0.8*sin(2*pi*0.5*id_time) + 0.4*sin(2*pi*2*id_time);
varr_data = collect_pcode_response(varr_volts, id_time, opts, params, SS_values, const_voltage_to_yaw_model);

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

%% Part B: controller synthesis + verification
plot_config.show_plots = opts.show_partB_plots;
plot_config.figure_prefix = opts.figure_prefix;

controls = controller_dev(params, opts.controller_vel, SS_values, plot_config);
results.controller = controls;

%% Part C: waypoint tracking with cascaded controller
partC_opts = struct('Ts', opts.Ts, ...
                    'sim_duration', opts.partC_duration, ...
                    'Vel', opts.controller_vel, ...
                    'waypoint_file', opts.partC_waypoint, ...
                    'voltage_limit', opts.voltage_limit, ...
                    'steering_limit_deg', opts.steering_limit_deg);

results.partC = solve_part_c(controls, params, partC_opts);

%% Part C validation: heading step response on the p-code plant
step_opts = struct('Ts', opts.Ts, ...
                  'sim_duration', 3, ...
                  'step_time', opts.step_time, ...
                  'step_deg', opts.step_deg, ...
                  'speeds', opts.step_speeds, ...
                  'voltage_limit', opts.voltage_limit, ...
                  'steering_limit_deg', opts.steering_limit_deg, ...
                  'yaw_tf', const_voltage_to_yaw_model);

results.step_response = part_c_step_response(controls, params, step_opts);

end


%%%%%%%%%%%%%%%%%%%
%%% Local Functions
%%%%%%%%%%%%%%%%%%%

function amp = steady_state_amplitude(signal, time_vector, freq_hz)
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

function merged = merge_structs(base, override)
    merged = base;
    names = fieldnames(override);
    for k = 1:numel(names)
        f = names{k};
        if isstruct(override.(f))
            merged.(f) = merge_structs(base.(f), override.(f));
        else
            merged.(f) = override.(f);
        end
    end
end

function val = get_option(opts, name, default)
    if isfield(opts, name)
        val = opts.(name);
    else
        val = default;
    end
end