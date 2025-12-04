function data = collect_pcode_response(voltage, time, opts, params, SS_values, yaw_model)

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
        const_rack_model = steering(params, SS_values);
        yaw_tf = build_bicycle_model(params, opts.Vel);
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