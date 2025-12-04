function [yaw_tf] = build_bicycle_model(params, Vel)
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