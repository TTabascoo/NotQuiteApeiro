package org.firstinspires.ftc.teamcode;
import static com.bylazar.telemetry.PanelsTelemetry.INSTANCE;

import static org.firstinspires.ftc.teamcode.shooterConstants.txRotationConstant;
import static java.lang.Math.atan2;
import static java.lang.Math.toDegrees;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
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
}
