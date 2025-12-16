package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.feedback.PIDCoefficients;

@Config
public class shooterConstants {

    private static double p;
    private static double i;
    private static double d;
    public static PIDCoefficients coefficients = new PIDCoefficients(p, i, d);
    public static double target;

}
