package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.constants.shooterConstants.txRotationConstant;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.autoPathConstants;
import org.firstinspires.ftc.teamcode.subsystems.driveTrain;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.locker;
import org.firstinspires.ftc.teamcode.subsystems.shooter;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.CommandManager;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "tuning TX")
public class tuningTX extends NextFTCOpMode {
    private boolean driveActive;
    private boolean turnOn;
    private Limelight3A limelight;
    private FieldManager panelsField = PanelsField.INSTANCE.getField();
    private double holdingPoseX;

    private double holdingPoseY;
    private double holdingX;


    @Override
    public void onStartButtonPressed() {
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
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.start();
        limelight.pipelineSwitch(0);
        follower().setStartingPose(autoPathConstants.pose);

    }

    @Override
    public void onUpdate() {
        BindingManager.update();
        follower().update();
        LLResult result = limelight.getLatestResult();

        if (gamepad1.leftBumperWasPressed()) {
            driveActive = false;
            turnOn = true;
            CommandManager.INSTANCE.cancelCommand(driveTrain.INSTANCE.driveControl2);
            BindingManager.update();
        }
        if (turnOn) {
            if (result != null && result.isValid()) {

                double tx = result.getTx();
                double ty = result.getTy();
                double ta = result.getTa();
                boolean isLeft = tx < 0;
                follower().turn(Math.toRadians(tx * txRotationConstant), isLeft);
                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
                telemetry.addData("rotation amount deg", txRotationConstant*tx);
                telemetry.update();
            } else {
                telemetry.addData("Limelight", "No Targets");
                telemetry.update();
            }
        }


        if (gamepad1.rightBumperWasPressed()) {
            turnOn = false;
            CommandManager.INSTANCE.scheduleCommand(driveTrain.INSTANCE.driveControl2);
            driveActive = true;
            follower().breakFollowing();
            BindingManager.update();
        }
        telemetry.addData("is turning?", follower().isTurning());
        telemetry.update();
    }
}
