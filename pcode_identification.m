function SS_values = pcode_identification(voltage, time, opts, params)
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