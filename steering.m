function G = steering(params, coefficients)
    s = tf('s');
    N = params.gear.N;
    Kt = params.vehicle.Kt;
    Je = coefficients.Je;
    Be = coefficients.Be;

    G = (N*Kt)/(Je*s^2 + Be*s);
end