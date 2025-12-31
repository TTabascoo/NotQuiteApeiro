//package org.firstinspires.ftc.teamcode;
//
//import static org.firstinspires.ftc.teamcode.autoPathConstants.headingStart;
//import static org.firstinspires.ftc.teamcode.autoPathConstants.pickUpAngle;
//import static org.firstinspires.ftc.teamcode.autoPathConstants.scoreAngle;
//
//import com.pedropathing.geometry.BezierLine;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
//
//import dev.nextftc.core.commands.Command;
//import dev.nextftc.core.commands.groups.ParallelGroup;
//import dev.nextftc.core.commands.groups.SequentialGroup;
//import dev.nextftc.core.components.SubsystemComponent;
//import dev.nextftc.extensions.pedro.FollowPath;
//import dev.nextftc.extensions.pedro.PedroComponent;
//import static dev.nextftc.extensions.pedro.PedroComponent.follower;
//
//import dev.nextftc.ftc.NextFTCOpMode;
//
//@Autonomous(name = "Auton (start at bottom, shoot in middle)")
//public class auto extends NextFTCOpMode {
//
//    public final Pose startPose = new Pose(56, 8, headingStart);
//    public final Pose scorePose = new Pose(60, 101, scoreAngle );
//    private final Pose pickUpPose1 = new Pose(43, 36, pickUpAngle);
//    private final Pose pickUpPose2 = new Pose(9, 36, pickUpAngle);
//    private PathChain score1;
//    private PathChain pickUpPath1;
//    private PathChain pickUp;
//    private PathChain scorePath1;
//    private PathChain score2;
//
//    public auto() {
//        addComponents(
//                new PedroComponent(Constants::createFollower),
//                new SubsystemComponent(shooter.INSTANCE),
//                new SubsystemComponent(intake.INSTANCE)
//        );
//    }
//    @Override
//    public void onInit() {
//        buildPaths();
//        follower().setStartingPose(startPose);
//    }
//
//    @Override
//    public void onStartButtonPressed() {
//    }
//
//    @Override
//    public void onUpdate() {
//        follower().update();
//        autoFull().schedule();
//        telemetry.addData("x", follower().getPose().getX());
//        telemetry.addData("y", follower().getPose().getY());
//        telemetry.addData("heading", follower().getPose().getHeading());
//        telemetry.addData("shooter1 val: ", shooter.INSTANCE.getPower1());
//        telemetry.addData("intake val: ", intake.INSTANCE.power());
//        telemetry.update();
//    }
//    public Command shoot1(double seconds) {
//        return new SequentialGroup(
//                new ParallelGroup(shooter.INSTANCE.shoot(seconds), intake.INSTANCE.autoRamp(seconds)
//                )
//        )
//                ;
//    }
//    public Command pickUp(double seconds) {
//        return new SequentialGroup(
//                intake.INSTANCE.autoRamp(seconds)
//        );
//    }
//    public Command moveScore1() {
//        return new FollowPath(score1);
//    }
//    public Command moveSetPickUp() {
//        return new FollowPath(pickUpPath1);
//    }
//    public Command movePickUp() {
//        return new FollowPath(pickUp);
//    }
//    public Command avoidBalls() {
//        return new FollowPath(scorePath1);
//    }
//    public Command moveScore2() {
//        return new FollowPath(score2);
//    }
//
//
//    public Command autoFull() {
//        return new SequentialGroup(
//                moveScore1(),
//                shoot1(2),
//                moveSetPickUp(),
//                movePickUp().and(pickUp(2)),
//                avoidBalls(),
//                moveScore2(),
//                shoot1(2)
//        );
//
//    }
//
//    private void buildPaths() {
//        score1 = follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(startPose, scorePose)
//                )
//                .setLinearHeadingInterpolation(headingStart, scoreAngle)
//                .build();
//
//        pickUpPath1 = follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(scorePose, pickUpPose1)
//                )
//                .setLinearHeadingInterpolation(scoreAngle, pickUpAngle)
//                .build();
//
//        pickUp = follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(pickUpPose1, pickUpPose2)
//                )
//                .setConstantHeadingInterpolation(pickUpAngle)
//                .build();
//
//        scorePath1 = follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(pickUpPose2, pickUpPose1)
//                )
//                .setTangentHeadingInterpolation()
//                .build();
//
//        score2 = follower()
//                .pathBuilder()
//                .addPath(
//                        new BezierLine(pickUpPose1, scorePose)
//                )
//                .setLinearHeadingInterpolation(pickUpAngle, scoreAngle)
//                .build();
//    }
// }





