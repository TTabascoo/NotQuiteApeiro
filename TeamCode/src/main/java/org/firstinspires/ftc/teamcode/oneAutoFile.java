package org.firstinspires.ftc.teamcode;

import static com.bylazar.telemetry.PanelsTelemetry.INSTANCE;
import static org.firstinspires.ftc.teamcode.autoPathConstants.headingStart;
import static org.firstinspires.ftc.teamcode.autoPathConstants.pickUpAngle;
import static org.firstinspires.ftc.teamcode.autoPathConstants.pickUpPose2;
import static org.firstinspires.ftc.teamcode.autoPathConstants.*;
import static org.firstinspires.ftc.teamcode.shooterConstants.target;
import static java.lang.Math.toDegrees;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class oneAutoFile {
    public static Command fullshoot(double acceleration, double lockerDelay, double shootingTime) {
        return new SequentialGroup(
                shooter.INSTANCE.shootCommand(),
                new Delay(acceleration),
                intake.INSTANCE.rampOn(1), //start running the ramp up
                new Delay(lockerDelay),
                locker.INSTANCE.open(),
                new Delay(shootingTime),// delay to shoot
                locker.INSTANCE.close()
        );
    }
    public static void buildPaths() {
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
        finalPath = follower()
                .pathBuilder()
                .addPath(new BezierLine(scorePose, finalPose)
                )
                .setLinearHeadingInterpolation(scoreAngle, finalAngle)
                .build();
    }
    public static void buildRightPath() {
        rightPath = follower().pathBuilder()
                .addPath(
                        new BezierLine(startPose, rightPose)
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(90))
                .build();
    }


    //COMMANDS THAT COMBINE BOTH INTAKE AND SHOOTER
    public static Command stopRamp() {
        return new SequentialGroup(
                shooter.INSTANCE.stopCommand2().and(intake.INSTANCE.rampOff())
        );
    }

    @Autonomous(name = "Auton 9 BLUE ONE FILE")
    public static class blueAuto extends NextFTCOpMode {
        private final TelemetryManager panelsTelemetry = INSTANCE.getTelemetry() ;

        public blueAuto() {
            addComponents(
                    new PedroComponent(Constants::createFollower),
                    new SubsystemComponent(shooter.INSTANCE),
                    new SubsystemComponent(intake.INSTANCE),
                    new SubsystemComponent(locker.INSTANCE),
                    BulkReadComponent.INSTANCE,
                    CommandManager.INSTANCE
            );
        }
        @Override
        public void onInit() {
            startPose = startPoseNow;
            scorePose = scorePoseNow;
            pickUpPose1 = pickUpPose1Now;
            pickUpPose2 = pickUpPose2Now;
            pickUpPose3 = pickUpPose3Now;
            pickUpPose4 = pickUpPose4now;
            finalPose = finalPoseNow;

            buildPaths();
            follower().setStartingPose(startPose);
            shooter.INSTANCE.stop(); //SCHEDULE STOPS TO MAKE SURE IT STOPS RUNNING EVERY INIT
            intake.INSTANCE.rampOff().schedule();
        }

        @Override
        public void onUpdate() {
            follower().update();
            panelsTelemetry.addData("shooter target", target);
            panelsTelemetry.addData("actual vel", shooter.INSTANCE.getVelocity());
            panelsTelemetry.update();
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

        //FULL AUTO COMMAND:
        public Command fullAuto() {
            return new SequentialGroup(
                    new FollowPath(score1),
                    fullshoot(0.9, 1.2,4),
                    stopRamp(),
                    new FollowPath(pickUpPath1),
                    new FollowPath(pickUp).and(intake.INSTANCE.rampOn(1)),
                    intake.INSTANCE.rampOff(),
                    new FollowPath(score2),
                    fullshoot(0.9, 1.2, 4),
                    new FollowPath(pickupPath2),
                    new FollowPath(pickUp2).and(intake.INSTANCE.rampOn(4)),
                    new FollowPath(score3),
                    fullshoot(0.9, 1.2, 5),
                    new FollowPath(finalPath));
        }

        //PATH STUFF



    }

    @Autonomous(name = "Auton 9 RED ONE FILE")
    public static class redAuto extends NextFTCOpMode {

        public redAuto() {
            addComponents(
                    new PedroComponent(Constants::createFollower),
                    new SubsystemComponent(shooter.INSTANCE),
                    new SubsystemComponent(intake.INSTANCE),
                    new SubsystemComponent(locker.INSTANCE),
                    BulkReadComponent.INSTANCE
            );
        }

        @Override
        public void onInit() {
            startPose = mirror(startPoseNow);
            scorePose = mirror(scorePoseNow);
            pickUpPose1 = mirror(pickUpPose1Now);
            pickUpPose2 = mirror(pickUpPose2Now);
            pickUpPose3 = mirror(pickUpPose3Now);
            pickUpPose4 = mirror(pickUpPose4now);
            finalPose = mirror(finalPoseNow);
            buildPaths();
            follower().setStartingPose(startPose);
            shooter.INSTANCE.stop();
            intake.INSTANCE.rampOff().schedule();
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
        public Command fullAuto() {
            return new SequentialGroup(
                    new FollowPath(score1), //follow the path to score1
                    fullshoot(0.5, 0.2, 3),
                    stopRamp(), //stop both the intake and shooter
                    new FollowPath(pickUpPath1), //follow this path
                    new FollowPath(pickUp).and(intake.INSTANCE.rampOn(1)),//follow the path and turn the ramp up
                    intake.INSTANCE.rampOff(), //stop intake after it finishes the path before
                    new FollowPath(score2), //follow the score2 path
                    fullshoot(0.5, 0.2, 3), //shoot again
                    new FollowPath(pickupPath2), // go towards balls
                    new FollowPath(pickUp2).and(intake.INSTANCE.rampOn(1)), // again follow path and turn on intake
                    intake.INSTANCE.rampOff(), // stop the thing
                    new FollowPath(score3), //go to score spot again
                    fullshoot(0.5 ,0.2, 3) //score again
            );
        }

        private Pose mirror(Pose originalPose) {
            double newX;
            double newHeading;
            newX = 144 - originalPose.getX();
            if(originalPose.getHeading() <= 180) {
                newHeading = 180 - toDegrees(originalPose.getHeading());
            } else {
                newHeading = toDegrees(originalPose.getHeading())-180;
            }
            return new Pose(newX, originalPose.getY(), newHeading);
        }

    }
    @Autonomous(name = "blue left auto")
    public static class RightAuto extends NextFTCOpMode {
        public RightAuto() {
            addComponents(
                    new PedroComponent(Constants::createFollower),
                    BulkReadComponent.INSTANCE
            );
        }
        @Override
        public void onInit() {
            startPose = new Pose(52, 8);
            rightPose = new Pose(22, 8);
            follower().setStartingPose(startPose);
            buildRightPath();
        }
        @Override
        public void onStartButtonPressed() {
            fullAutoRight().schedule();
        }
        public Command fullAutoRight() {
            return new SequentialGroup(
                    new FollowPath(rightPath)
            );
        }
        @Override
        public void onUpdate() {
            follower().update();
        }

    }
    @Autonomous(name = "red right auto")
    public static class LeftAuto extends NextFTCOpMode {
        public LeftAuto() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

        @Override
        public void onInit() {
            startPose = new Pose(84, 8);
            rightPose = new Pose(115, 8);
            buildRightPath();
            follower().setPose(startPose);
        }
        @Override
        public void onStartButtonPressed() {
            rightPathAuto().schedule();
        }
        public Command rightPathAuto() {
            return new SequentialGroup(
                    new FollowPath(rightPath)
            );
        }

        @Override
        public void onUpdate() {
            follower().update();
        }
    }
}
