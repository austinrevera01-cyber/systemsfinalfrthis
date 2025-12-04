function controller = controller_dev(params, velocity, SS_values, plot_opts)

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

    [ref_amp, ref_phase]   = fit_sine(sine_eval.psi_ref, sine_eval.time, sine_eval.freq_rad_s);
    [resp_amp, resp_phase] = fit_sine(sine_eval.psi_resp, sine_eval.time, sine_eval.freq_rad_s);
    sine_eval.amp_ratio    = resp_amp / ref_amp;
    sine_eval.phase_lag_deg = wrap_to_180(rad2deg(resp_phase - ref_phase));

    [bode_mag, bode_phase, ~] = bode(T_psi, sine_eval.freq_rad_s);
    sine_eval.bode_mag        = squeeze(bode_mag);
    sine_eval.bode_phase_deg  = squeeze(bode_phase);

    controller.sine_eval = sine_eval;

    if plot_opts.show_plots
        prefix = plot_opts.figure_prefix;
        quick_step_plot(T_delta, [prefix 'Inner steering: \delta_{ref} -> \delta']);
        quick_step_plot(T_r, [prefix 'Yaw-rate loop: r_{ref} -> r']);
        quick_step_plot(T_psi, [prefix 'Heading loop: \psi_{ref} -> \psi']);

        pole_plot(controller.loops.delta.poles, [prefix 'Steering loop poles']);
        pole_plot(controller.loops.r.poles, [prefix 'Yaw-rate loop poles']);
        pole_plot(controller.loops.psi.poles, [prefix 'Heading loop poles']);

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
function quick_step_plot(T, title_str)
    figure('Name', title_str);
    step(T);
    grid on;
    title(title_str);
end

function pole_plot(poles, title_str)
    figure('Name', title_str);
    plot(real(poles), imag(poles), 'x', 'MarkerSize', 10, 'LineWidth', 2);
    xlabel('Real'); ylabel('Imag'); grid on; title(title_str);
end

function [amp, phase] = fit_sine(signal, time_vector, freq_rad_s)
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

function deg_wrapped = wrap_to_180(degrees)
    % Wrap angles to the [-180, 180] deg range for reporting phase lags.
    deg_wrapped = mod(degrees + 180, 360) - 180;
end