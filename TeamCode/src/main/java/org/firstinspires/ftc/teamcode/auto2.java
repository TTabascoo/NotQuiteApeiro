package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.autoPathConstants.headingStart;
import static org.firstinspires.ftc.teamcode.autoPathConstants.pickUpAngle;
import static org.firstinspires.ftc.teamcode.autoPathConstants.scoreAngle;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;


import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auton 9 (start at top, shoot in middle)")
public class auto2 extends NextFTCOpMode {

    public auto2() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(shooter.INSTANCE),
                new SubsystemComponent(intake.INSTANCE)
        );
    }
    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
    }

    @Override
    public void onUpdate() {
        follower().update();
        fullAuto().schedule();
        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
        telemetry.addData("shooter val: ", shooter.INSTANCE.power());
        telemetry.addData("intake val: ", intake.INSTANCE.power());
        telemetry.update();
    }

    public Command shoot1() {
        return new SequentialGroup(
                new ParallelGroup(shooter.INSTANCE.shoot, intake.INSTANCE.upRamp
                ), new Delay(5)
        )
                ;
    }
    public Command pickUp() {
        return new SequentialGroup(
                intake.INSTANCE.upRamp, new Delay(1)
        );
    }

    public Command fullAuto() {
        return new SequentialGroup(
                new FollowPath(score1),
                shoot1(),
                new FollowPath(pickUpPath1),
                new FollowPath(pickUp).and(pickUp()),
                new FollowPath(score2),
                new FollowPath(pickupPath2),
                new FollowPath(pickUp2).and(pickUp()),
                new FollowPath(score3),
                shoot1()
        );
    }
    public PathChain score1;
    public PathChain pickUpPath1;
    public PathChain pickUp;
    public PathChain pickUp2;
    public PathChain pickupPath2;
    public PathChain score2;
    public PathChain score3;


    private final Pose startPose = new Pose(33, 135, headingStart);
    private final Pose scorePose = new Pose(60, 101, scoreAngle );
    private final Pose pickUpPose1 = new Pose(42, 84, pickUpAngle);
    private final Pose pickUpPose2 = new Pose(14, 84, pickUpAngle);
    private final Pose pickUpPose3 = new Pose(42, 60, pickUpAngle);
    private final Pose pickUpPose4 = new Pose(14, 60, pickUpAngle);


    private void buildPaths() {
            score1 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(startPose, scorePose)
                    )
                    .setLinearHeadingInterpolation(headingStart, scoreAngle)
                    .build();

            pickUpPath1 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(scorePose, pickUpPose1)
                    )
                    .setLinearHeadingInterpolation(scoreAngle, pickUpAngle)
                    .build();

            pickUp = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUpPose1, pickUpPose2)
                    )
                    .setConstantHeadingInterpolation(pickUpAngle)
                    .build();

            score2 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUpPose2, scorePose)
                    )
                    .setLinearHeadingInterpolation(pickUpAngle, scoreAngle)
                    .build();

            pickupPath2 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(scorePose, pickUpPose3)
                    )
                    .setLinearHeadingInterpolation(scoreAngle, pickUpAngle)
                    .build();

            pickUp2 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUpPose3, pickUpPose4)
                    )
                    .setConstantHeadingInterpolation(pickUpAngle)
                    .build();

            score3 = follower()
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pickUpPose4, scorePose)
                    )
                    .setLinearHeadingInterpolation(pickUpAngle, scoreAngle)
                    .build();
        }

}

