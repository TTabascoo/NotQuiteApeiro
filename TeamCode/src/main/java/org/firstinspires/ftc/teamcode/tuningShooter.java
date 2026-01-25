package org.firstinspires.ftc.teamcode;
import static com.bylazar.telemetry.PanelsTelemetry.*;
import static org.firstinspires.ftc.teamcode.constants.shooterConstants.actualVelocity;
import static org.firstinspires.ftc.teamcode.constants.shooterConstants.target;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import org.firstinspires.ftc.teamcode.constants.shooterConstants;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class tuningShooter extends NextFTCOpMode {
    private DcMotorEx motorShooter;
    private DcMotorEx motorShooter2;
    private ControlSystem controller;
    private double powerNeeded;
    private TelemetryManager panelsTelemetry = INSTANCE.getTelemetry() ;

    @Override
    public void onInit() {
        motorShooter = hardwareMap.get(DcMotorEx.class, "shooter");
        motorShooter2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        controller = ControlSystem.builder()
                .velPid(shooterConstants.coefficients)
                .basicFF(shooterConstants.ffcoefficients)
                .build();
    }

    @Override
    public void onUpdate() {
        powerNeeded =controller.calculate(new KineticState(
                motorShooter.getCurrentPosition() ,
                motorShooter.getVelocity()
        ));
        if (gamepad1.bWasPressed()) {
            controller.setGoal(new KineticState(0, 1750, 0));
        }
        if(gamepad1.aWasPressed()) {
            controller.setGoal(new KineticState(0, 0, 0));
        }

        motorShooter.setPower(powerNeeded);
        motorShooter2.setPower(-powerNeeded);

        actualVelocity = motorShooter.getVelocity();

        panelsTelemetry.addData("target vel", target);
        panelsTelemetry.addData("actual vel", actualVelocity);
        panelsTelemetry.update();
        telemetry.addData("target vel", target);
        telemetry.addData("actual vel", actualVelocity);
        telemetry.addData("goal", controller.getGoal());


    }
}
