package org.firstinspires.ftc.teamcode;
import static com.bylazar.telemetry.PanelsTelemetry.INSTANCE;

import static org.firstinspires.ftc.teamcode.constants.shooterConstants.txRotationConstant;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.autoPathConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.locker;
import org.firstinspires.ftc.teamcode.subsystems.shooter;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;

import dev.nextftc.ftc.NextFTCOpMode;

import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Full Teleop, Click Here!")
public class teleOp extends NextFTCOpMode {
    public teleOp() {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(locker.INSTANCE),
                new SubsystemComponent(shooter.INSTANCE),
                new SubsystemComponent(driveTrain.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                CommandManager.INSTANCE
        );
    }
    private final TelemetryManager panelsTelemetry = INSTANCE.getTelemetry() ;
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();
    public Limelight3A limelight;
    private double angleForScoring;
    private Pose holdingPose;
    private PathChain selfPath;
    private boolean turnOn;
    public boolean driveActive;
    public static double holdingPoseX;
    public static double holdingPoseY;
    public static double holdingPoseHeading;
    public double angle;
    public double turnAngle;
    public Pose goalPose;



    @Override
    public void onStartButtonPressed() {
        //reset binding manager, button maps
        driveActive = true;
        BindingManager.reset();
        shooter.INSTANCE.buttonMap();
        intake.INSTANCE.buttonMap();
        locker.INSTANCE.buttonMap();
        driveTrain.INSTANCE.driveControl2.schedule();
        driveTrain.INSTANCE.driveControl2.setScalar(1);

        BindingManager.update();
    }
    @Override
    public void onWaitForStart() {
        //set the goal pose to be used later
        //TODO MAKE IT SO U CAN CHANGE FROM BLUE to RED GOALS, OR ADD ANOTHER TELEOP
        goalPose = autoPathConstants.blueGoalPose;
//        telemetry.addLine("share for red!");
//        telemetry.addLine("options for blue!");
//        telemetry.update();
//        if (gamepad1.shareWasPressed()) {
//            follower().setStartingPose(new Pose(0, 0, Math.toRadians(0)));
//            angleForScoring = Math.toRadians(215);
//            telemetry.clear();
//            telemetry.addLine("red selected!");
//            telemetry.update();
//        }
//        if(gamepad1.optionsWasPressed()) {
//            angleForScoring = Math.toRadians(300);
//            follower().setStartingPose(new Pose(0, 0, Math.toRadians(180)));
//            telemetry.clear();
//            telemetry.addLine("blue slelected1");
//            telemetry.update();
//        }
//        telemetry.addData("angle", angleForScoring);
    }


    @Override
    public void onInit() {
        //ON INIT: stop shooter, hardware map LL (no subsystem class made yet)
        //start LL -> look at docs idk what to tell you here
        shooter.INSTANCE.stop();
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(1);
        //set up field
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
        //set starting pose as the global pose
        follower().setStartingPose(autoPathConstants.pose);

    }

    @Override
    public void onUpdate() {
        //update bindingmanager -> able to actually use them
        //update follower -> constantly updating position
        //draw follower
        BindingManager.update();
        follower().update();
        drawDebug(follower());
        //if left bumper press -> cancel driving command, driveActive false -> dt is not on
        //update the bindingmanager to store change (idk if necessary)
        if(gamepad1.leftBumperWasPressed()) {
            driveActive = false;
            CommandManager.INSTANCE.cancelCommand(driveTrain.INSTANCE.driveControl2);
            BindingManager.update();
        }
        //if the drive active is false and localizer is not null
        // grab the pose constantly as well as the atan two of it
        // create a path that goes to the same follower x y and with angle atan2 stuff (yay math)
        //if you actually read this i owe u 32 cents
        if(!driveActive && !follower().isLocalizationNAN()) {
            holdingPoseX = follower().getPose().getX();
            holdingPoseY = follower().getPose().getY();
            holdingPoseHeading = Math.toDegrees(follower().getPose().getHeading());
            angle = Math.atan2(goalPose.getY()-holdingPoseY, goalPose.getX()-holdingPoseX);

            selfPath = follower().pathBuilder().addPath(new BezierPoint(new Pose(holdingPoseX, holdingPoseY))).setConstantHeadingInterpolation(angle).build();
            follower().followPath(selfPath, true);
        }

        //if the right bumper was pressed
        //resechedule dt and make drive active true again
        //break following to stop following that path
        if(gamepad1.rightBumperWasPressed()) {
            CommandManager.INSTANCE.scheduleCommand(driveTrain.INSTANCE.driveControl2);
            driveActive = true;
            follower().breakFollowing();
            BindingManager.update();
        }

//        if(!driveActive) {
//            LLResult result = limelight.getLatestResult();
//
//            if (result != null && result.isValid()) {
//
//                double tx = result.getTx();
//                double ty = result.getTy();
//                double ta = result.getTa();
//
//                double x = result.getBotpose().getPosition().x;
//                double y = result.getBotpose().getPosition().y;
//
//                double heading = result.getBotpose().getOrientation().getYaw();
//                Pose updatedPose = new Pose(x, y, heading);
//                Pose ftcUpdated = updatedPose.getAsCoordinateSystem(InvertedFTCCoordinates.INSTANCE);
//                Pose pedroPose = ftcUpdated.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//
//
//                follower().setPose(pedroPose);
//                telemetry.addData("Target X", tx);
//                telemetry.addData("Target Y", ty);
//                telemetry.addData("Target Area", ta);
//                telemetry.update();
//            } else {
//                telemetry.addData("Limelight", "No Targets");
//                telemetry.update();
//            }
//
//        }


//
//        if (gamepad1.left_bumper) {
//            follower().turnTo(Math.toRadians(angleForScoring));
//            driveTrain.INSTANCE.driveControlOff();
//
//            double holdingX = follower().getPose().getX();
//            double holdingY = follower().getPose().getY();
//            double holdingYaw = follower().getPose().getHeading();
//            follower().holdPoint(new Pose(holdingX, holdingY, holdingYaw));
//        }
////        if(follower().isTurning() || follower().isBusy()) {
////            driveTrain.INSTANCE.driveControlOff();
////        } else {
////            driveTrain.INSTANCE.driveControlOn();
////        }
//
//        if(gamepad1.rightBumperWasPressed()) {
//            driveTrain.INSTANCE.driveControlOn();
//            follower().breakFollowing();
//        }
//        follower().update();
        // telemetry stuff
        telemetry.addData("drive active ? ", driveActive);
        telemetry.addData("command", CommandManager.INSTANCE.snapshot());
        telemetry.addData("follower x", follower().getPose());
        telemetry.addData("holding pose", holdingPose);
        telemetry.addData("holding angle", angleForScoring);
        telemetry.addData("goal pose", goalPose);

        telemetry.addData("is turning", follower().isTurning());
        telemetry.addData("is busy ", follower().isBusy());
        telemetry.addData("current path", follower().getCurrentPath());

        telemetry.update();
        panelsTelemetry.addData("tx rotation constant", txRotationConstant);
        panelsTelemetry.addData("actual vel", shooter.INSTANCE.getVelocity());
        panelsTelemetry.addData("target,", shooter.INSTANCE.getTarget());
        panelsTelemetry.update();


    }
    @Override
    public void onStop() {
        BindingManager.reset();
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


