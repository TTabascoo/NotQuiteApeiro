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

import java.time.Duration;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
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
        telemetry.addData("does it work pease", follower().isBusy());
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

    public Command fullshoot(double delay, double delay2, double shootingTime) {
        return new SequentialGroup(shooter.INSTANCE.shoot(1), //start running the shooter first to accelarate
                new Delay(delay), //small delay before running intake
                intake.INSTANCE.autoRamp(1), //start running the ramp up
                new Delay(delay2),
                locker.INSTANCE.open(),
                new Delay(shootingTime),// delay to shoot
                locker.INSTANCE.close()
        );
    }
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
                score1Path(),
                pause(),
                fullshoot(0.3, 0.2,5),
                stopRamp(),
                resume(),
                pickUpPathSetUp1(),
                pickUpPath().and(intake.INSTANCE.autoRamp(1)),
                intake.INSTANCE.stop(),
                score2Path(),
                fullshoot(0.3, 0.2, 5),
                pickUpPathSetUp2(),
                pickUp2().and(intake.INSTANCE.autoRamp(1)),
                score3Path(),
                fullshoot(0.3, 0.2, 5))
                ;
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

