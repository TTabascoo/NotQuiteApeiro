package org.firstinspires.ftc.teamcode.autos;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autoPathConstants;
import org.firstinspires.ftc.teamcode.intake;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.shooter;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

//TESTED BLUE FAR MIRRORED (NOT VIZ ACCURATE)
@Autonomous(name = "red auto far")
public class redAutoFar extends NextFTCOpMode {
    public redAutoFar() {
        addComponents(
                new SubsystemComponent(shooter.INSTANCE, intake.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE
        );
    }
    private final FieldManager panelsField = PanelsField.INSTANCE.getField();
    private Paths paths;
    @Override
    public void onInit() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        shooter.INSTANCE.setTarget(2050);

        paths = new Paths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(88, 8, Math.toRadians(-90)));
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
                new InstantCommand(()-> shooter.INSTANCE.shoot()),
                new WaitUntil(() -> shooter.INSTANCE.reachedTarget()),
                intake.INSTANCE.rampOn(0.8),
                new Delay(3),
                new InstantCommand(() -> shooter.INSTANCE.stop()),
                intake.INSTANCE.rampOff(),
                new FollowPath(paths.Path2),
                new FollowPath(paths.Path3).and(intake.INSTANCE.rampOn(1)),
                intake.INSTANCE.rampOff(),
                new FollowPath(paths.Path4),
                new InstantCommand(()-> shooter.INSTANCE.shoot()),
                new WaitUntil(() -> shooter.INSTANCE.reachedTarget()),
                intake.INSTANCE.rampOn(0.8),
                new Delay(3),
                new InstantCommand(() -> shooter.INSTANCE.stop()),
                intake.INSTANCE.rampOff(),
                new FollowPath(paths.Path5),
                new FollowPath(paths.Path6).and(intake.INSTANCE.rampOn(1)),
                intake.INSTANCE.rampOff(),
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
        autoPathConstants.pose = PedroComponent.follower().getPose();
        drawDebug(PedroComponent.follower());
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

                                    new Pose(88.624, 11.241)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-116))

                    .build();

            Path2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.624, 11.241),

                                    new Pose(99.311, 34.774)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(0))

                    .build();

            Path3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(99.311, 34.774),

                                    new Pose(131.221, 34.669)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path4 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.221, 34.669),

                                    new Pose(88.769, 12.745)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-116))

                    .build();

            Path5 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.769, 12.745),

                                    new Pose(97.285, 60.122)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(-116), Math.toRadians(0))

                    .build();

            Path6 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(97.285, 60.122),

                                    new Pose(131.103, 59.753)
                            )
                    ).setConstantHeadingInterpolation(Math.toRadians(0))

                    .build();

            Path7 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(131.103, 59.753),

                                    new Pose(88.388, 13.240)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-116))

                    .build();
        }
    }

    public static final double ROBOT_RADIUS = 9; // woah

    private final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */


    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public void sendPacket() {
        panelsField.update();
    }
}
