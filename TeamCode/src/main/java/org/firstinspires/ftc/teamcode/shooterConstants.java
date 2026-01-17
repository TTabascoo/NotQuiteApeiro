package org.firstinspires.ftc.teamcode;


import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.control.feedforward.FeedforwardElement;

@Configurable
public class shooterConstants {

    private static double p = 0.003;
    private static final double i = 0;
    private static final double d = 0;
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    public static double target = 1650;
    public static double shooterdirection = 1;
    public static double kV = 0.00043;    //TO DO Tune kV, kS, P
    public static double kA;
    public static double kS;
    public static BasicFeedforwardParameters ffcoefficients = new BasicFeedforwardParameters(kV, kA, kS);

    public static double actualVelocity;
    public static double shootingSpotVel;
    public static double threshold = 100;
    public static KineticState tolerance = new KineticState(Double.POSITIVE_INFINITY, threshold+target, Double.POSITIVE_INFINITY);
    public static double txRotationConstant;
}
