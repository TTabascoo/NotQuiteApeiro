package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "blue auto far")
public class auto4 extends NextFTCOpMode {
    public auto4() {
        addComponents(
                new SubsystemComponent(shooter.INSTANCE, intake.INSTANCE),
                new PedroComponent(Constants::createFollower)
        );
    }
    private Paths paths;
    @Override
    public void onInit() {
        shooter.INSTANCE.setTarget(2050);
        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(56, 8, Math.toRadians(90)));
        shooter.INSTANCE.stop();
        intake.INSTANCE.rampOff().schedule();
    }
    public Command fullShoot() {
        return new SequentialGroup(
                new InstantCommand(()-> shooter.INSTANCE.shoot()),
                new WaitUntil(() -> shooter.INSTANCE.reachedTarget()),
                intake.INSTANCE.rampOn(1),
                new Delay(3),
                new InstantCommand(() -> shooter.INSTANCE.stop()),
                intake.INSTANCE.rampOff()
        );
    }
    public Command fullAuto() {
        return new SequentialGroup(
                new FollowPath(paths.Path1),
                fullShoot(),
                new FollowPath(paths.Path2),
                new FollowPath(paths.Path3).and(intake.INSTANCE.rampOn(1)),
                new FollowPath(paths.Path4),
                fullShoot(),
                new FollowPath(paths.Path5),
                new FollowPath(paths.Path6).and(intake.INSTANCE.rampOn(1)),
                new FollowPath(paths.Path7),
                fullShoot()
        );
    }
    @Override
    public void onStartButtonPressed() {
        fullAuto().schedule();
    }
    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
    }

    public static class Paths {
        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;

        public Paths (Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(55.376, 11.241)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(106))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.376, 11.241),

                                    new Pose(39.993, 34.551)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(106), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(39.993, 34.551),

                                    new Pose(12.779, 34.669)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.779, 34.669),

                                    new Pose(55.376, 10.886)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(106))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(55.376, 10.886),

                                    new Pose(40.230, 60.345)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(106), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(40.230, 60.345),

                                    new Pose(12.897, 59.753)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(12.897, 59.753),

                                    new Pose(55.612, 11.004)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(106))

                    .build();
        }
    }
}
