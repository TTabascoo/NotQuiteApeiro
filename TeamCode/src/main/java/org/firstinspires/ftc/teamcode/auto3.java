package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.autoPathConstants.headingStart;
import static org.firstinspires.ftc.teamcode.autoPathConstants.pickUpAngle;
import static org.firstinspires.ftc.teamcode.autoPathConstants.scoreAngle;
import static org.firstinspires.ftc.teamcode.shooterConstants.shooterdirection;
import static java.lang.Math.toDegrees;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "Auton 9 RED")
public class auto3 extends NextFTCOpMode {

    public auto3() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(shooter.INSTANCE),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(locker.INSTANCE)
        );
    }

    @Override
    public void onInit() {
        buildPaths();
        follower().setStartingPose(startPose);
        shooter.INSTANCE.stop().schedule();
        intake.INSTANCE.stop().schedule();
    }

    @Override
    public void onUpdate() {
        follower().update();
        telemetry.addData("x", follower().getPose().getX());
        telemetry.addData("y", follower().getPose().getY());
        telemetry.addData("heading", follower().getPose().getHeading());
        telemetry.addData("shooter val: ", shooter.INSTANCE.getPower1());
        telemetry.addData("shooter val2: ", shooter.INSTANCE.getPower2());
        telemetry.addData("intake val: ", intake.INSTANCE.power());
        telemetry.addData("path", follower().getCurrentPath());
        telemetry.addData("does it work please", follower().isBusy());
        telemetry.update();
    }
    @Override
    public void onStartButtonPressed() {
        fullAuto().schedule();
    }



    public Command stopRamp() {
        return new SequentialGroup(
                shooter.INSTANCE.stop().and(intake.INSTANCE.stop())
        );
    }

    public Command fullshoot(double acceleration, double lockerDelay, double shootingTime) {
        return new SequentialGroup(
                shooter.INSTANCE.shoot(shooterdirection), //start running the shooter first to accelerate
                new Delay(acceleration), //small delay before running intake
                intake.INSTANCE.autoRamp(1), //start running the ramp up
                new Delay(lockerDelay),
                locker.INSTANCE.open(),
                new Delay(shootingTime),// delay to shoot
                locker.INSTANCE.close()
        );
    }

    //commands to make cleaner, can replace idm
    public Command pause() {
        return new InstantCommand(() -> follower().pausePathFollowing());
    }
    public Command resume() {
        return new InstantCommand(() -> follower().resumePathFollowing());
    }
    public Command score1Path() {
        return new FollowPath(score1);
    }
    public Command pickUpPathSetUp1() {
        return new FollowPath(pickUpPath1);
    }

    public Command pickUpPath() {
        return new FollowPath(pickUp);
    }

    public Command score2Path() {
        return new FollowPath(score2);
    }
    public Command pickUpPathSetUp2() {
        return new FollowPath(pickupPath2);
    }
    public Command pickUp2() {
        return new FollowPath(pickUp2);
    }
    public Command score3Path() {
        return new FollowPath(score3);
    }

    public Command fullAuto() {
        return new SequentialGroup(
                score1Path(), //follow the path to score1
                pause(), //pause the path following, might be able to remove this
                fullshoot(0.5, 0.2, 3),
                stopRamp(), //stop both the intake and shooter
                resume(), //resume the path following
                pickUpPathSetUp1(), //follow this path
                pickUpPath().and(intake.INSTANCE.autoRamp(1)),//follow the path and turn the ramp up
                intake.INSTANCE.stop(), //stop intake after it finishes the path before
                score2Path(), //follow the score2 path
                fullshoot(0.5, 0.2, 3), //shoot again
                pickUpPathSetUp2(), // go towards balls
                pickUp2().and(intake.INSTANCE.autoRamp(1)), // again follow path and turn on intake
                intake.INSTANCE.stop(), // stop the thing
                score3Path(), //go to score spot again
                fullshoot(0.5 ,0.2, 3) //score again
        );
    }
    public PathChain score1;
    public PathChain pickUpPath1;
    public PathChain pickUp;
    public PathChain pickUp2;
    public PathChain pickupPath2;
    public PathChain score2;
    public PathChain score3;


    private final Pose startPose = mirror(new Pose(33, 135, headingStart));
    private final Pose scorePose = mirror(new Pose(60, 101, scoreAngle ));
    private final Pose pickUpPose1 = mirror(new Pose(42, 84, pickUpAngle));
    private final Pose pickUpPose2 = mirror(new Pose(14, 84, pickUpAngle));
    private final Pose pickUpPose3 = mirror(new Pose(42, 60, pickUpAngle));
    private final Pose pickUpPose4 = mirror(new Pose(14, 60, pickUpAngle));

    private Pose mirror(Pose originalPose) {
        double newX;
        double newHeading;
        newX = 144 - originalPose.getX();
        newHeading = 180 - toDegrees(originalPose.getHeading());
        return new Pose(newX, originalPose.getY(), newHeading);
    }



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

