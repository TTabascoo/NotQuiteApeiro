package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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

@Autonomous(name = "red auto far")
public class auto5 extends NextFTCOpMode {
    public auto5() {
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
        PedroComponent.follower().setStartingPose(new Pose(88, 8, Math.toRadians(90)));
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

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 8.000),

                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(69))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),

                                    new Pose(103.000, 35.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(180))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(103.000, 35.000),

                                    new Pose(130.000, 35.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 35.000),

                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(69))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(86.000, 15.000),

                                    new Pose(102.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(69), Math.toRadians(180))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 60.000),

                                    new Pose(130.000, 60.000)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(180))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 60.000),

                                    new Pose(86.000, 15.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(69))

                    .build();
        }
    }
}
