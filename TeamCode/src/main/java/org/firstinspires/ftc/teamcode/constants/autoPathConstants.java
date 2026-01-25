package org.firstinspires.ftc.teamcode.constants;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;

@Configurable
public class autoPathConstants {
    public static final double headingStart = Math.toRadians(90);
    public static final double scoreAngle = Math.toRadians(145);
    public static final double pickUpAngle = Math.toRadians(0);
    public static final double finalAngle = Math.toRadians(180);
    public static Pose pose = new Pose();
    public static final Pose blueGoalPose = new Pose(11, 138);


}
