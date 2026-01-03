package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;

@Configurable
public class shooterConstants {

    private static double p;
    private static final double i = 0;
    private static final double d = 0;
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    public static double target;
    public static double shooterdirection = 1;
    public static double kV;    //TO DO Tune kV, kS, P
    public static double kA = 0;
    public static double kS;
    public static double actualVelocity;
    public static ControlSystem controller;
    public static double shootingSpotVel;
}
