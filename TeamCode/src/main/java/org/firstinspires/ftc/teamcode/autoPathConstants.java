package org.firstinspires.ftc.teamcode;

import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public class autoPathConstants {
    public static final double headingStart = Math.toRadians(90);
    public static final double scoreAngle = Math.toRadians(326);
    public static final double pickUpAngle = Math.toRadians(180);
    public static PathChain score1;
    public static PathChain pickUpPath1;
    public static PathChain pickUp;
    public static PathChain pickUp2;
    public static PathChain pickupPath2;
    public static PathChain score2;
    public static PathChain score3;
    public static Pose startPose;
    public static Pose scorePose;
    public static Pose pickUpPose1;
    public static Pose pickUpPose2;
    public static Pose pickUpPose3;
    public static Pose pickUpPose4;
    public static Pose startPoseNow = new Pose(33, 135, headingStart);
    public static Pose scorePoseNow = new Pose(60, 101, scoreAngle );
    public static Pose pickUpPose1Now = new Pose(42, 84, pickUpAngle);
    public static Pose pickUpPose2Now = new Pose(20, 84, pickUpAngle);
    public static Pose pickUpPose3Now = new Pose(42, 60, pickUpAngle);
    public static Pose pickUpPose4now = new Pose(20, 60, pickUpAngle);


}
