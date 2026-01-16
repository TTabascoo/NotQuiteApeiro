package org.firstinspires.ftc.teamcode;
import static com.bylazar.telemetry.PanelsTelemetry.INSTANCE;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.Companion;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import java.lang.reflect.Array;

import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Full Teleop, Click Here!")
public class teleOp extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE, locker.INSTANCE, shooter.INSTANCE, driveTrain.INSTANCE),
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    PathChain selfPath;
    private TelemetryManager panelsTelemetry = INSTANCE.getTelemetry() ;
    private static FieldManager panelsField = PanelsField.INSTANCE.getField();
    public Limelight3A limelight;
    public boolean locked;


    @Override
    public void onStartButtonPressed() {
        shooter.INSTANCE.buttonMap();
        intake.INSTANCE.buttonMap();
        locker.INSTANCE.buttonMap();
        driveTrain.INSTANCE.driveControl().schedule();
    }


    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());

    }

    @Override
    public void onUpdate() {
        BindingManager.update();

        LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();

                Pose3D updatedPose3D = result.getBotpose();
                double x = updatedPose3D.getPosition().x;
                double y = updatedPose3D.getPosition().y;
                double heading = updatedPose3D.getOrientation().getYaw();
                Pose updatedPose = new Pose(x, y, heading);
                Pose pedroPose = updatedPose.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                follower().setPose(pedroPose);
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.update();
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }

        if (gamepad1.leftBumperWasPressed()) {
            follower().turnTo(Math.toRadians(145));

            double holdingX = follower().getPose().getX();
            double holdingY = follower().getPose().getY();
            double holdingYaw = follower().getPose().getHeading();
            follower().holdPoint(new Pose(holdingX, holdingY, holdingYaw));
        }
        if(follower().isTurning() || follower().isBusy()) {
            driveTrain.INSTANCE.driveControlOff();
        } else {
            driveTrain.INSTANCE.driveControlOn();
        }

        if(gamepad1.rightBumperWasPressed()) {
            follower().breakFollowing();
        }
        follower().update();
        panelsTelemetry.debug("x:" + Tuning.follower.getPose().getX());
        panelsTelemetry.debug("y:" + Tuning.follower.getPose().getY());
        panelsTelemetry.debug("heading:" + Tuning.follower.getPose().getHeading());
        panelsTelemetry.debug("total heading:" + Tuning.follower.getTotalHeading());
        panelsTelemetry.update(telemetry);
        draw();


        telemetry.addData("fr power", driveTrain.INSTANCE.getFRPower());
        telemetry.addData("fl power", driveTrain.INSTANCE.getFLPower());
        telemetry.addData("br power", driveTrain.INSTANCE.getBRPower());
        telemetry.addData("bl power", driveTrain.INSTANCE.getBLPower());
        telemetry.addData("intake power", intake.INSTANCE.power());
        telemetry.addData("intake direction", intake.INSTANCE.getDirection());
        telemetry.addData("shooter power 1", shooter.INSTANCE.getPower1());
        telemetry.addData("shooter power 2", shooter.INSTANCE.getPower2());
        telemetry.addData("shooter direction", shooter.INSTANCE.getDirection());
//        telemetry.addData("shooter goal", shooter.INSTANCE.getGoal());
        telemetry.addData("locker position", locker.INSTANCE.getPosition());
//        telemetry.addData("result valid ? ", limelightCamera.INSTANCE.validResults());
//        telemetry.addData("needed angle", limelightCamera.INSTANCE.getAngledNeeded());
//        telemetry.addData("tx", limelightCamera.INSTANCE.getTx());
//        telemetry.addData("ty", limelightCamera.INSTANCE.getTy());
        telemetry.update();
        panelsTelemetry.addData("actual vel", shooter.INSTANCE.getVelocity());
        panelsTelemetry.addData("target,", shooter.INSTANCE.getTarget());
        panelsTelemetry.update();


    }
    @Override
    public void onStop() {
        BindingManager.reset();
    }
    public static final double ROBOT_RADIUS = 9; // woah

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.75
    );

    public static void drawDebug(Follower follower) {
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
    public static void drawRobot(Pose pose, Style style) {
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
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
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
    public static void drawPath(PathChain pathChain, Style style) {
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
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
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
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
    public static void drawOnlyCurrent() {
        try {
            drawRobot(follower().getPose());
            sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void draw() {
        drawDebug(follower());
    }
}

