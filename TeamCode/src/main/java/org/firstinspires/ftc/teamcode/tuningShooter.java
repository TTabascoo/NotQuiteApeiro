package org.firstinspires.ftc.teamcode;
import static com.bylazar.telemetry.PanelsTelemetry.*;
import static org.firstinspires.ftc.teamcode.shooterConstants.actualVelocity;
import static org.firstinspires.ftc.teamcode.shooterConstants.kA;
import static org.firstinspires.ftc.teamcode.shooterConstants.kS;
import static org.firstinspires.ftc.teamcode.shooterConstants.kV;
import static org.firstinspires.ftc.teamcode.shooterConstants.target;

import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import com.bylazar.telemetry.PanelsTelemetry;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class tuningShooter extends NextFTCOpMode {
    private DcMotorEx motorShooter;
    private ControlSystem controller;
    private TelemetryManager panelsTelemetry = INSTANCE.getTelemetry() ;

    @Override
    public void onInit() {
        motorShooter = hardwareMap.get(DcMotorEx.class, "shooter");
        controller = ControlSystem.builder()
                .velPid(shooterConstants.coefficients)
                .basicFF(kV, kA, kS)
                .build();
    }

    @Override
    public void onUpdate() {
        controller.setGoal(new KineticState(0, target, 0));

        motorShooter.setPower(controller.calculate(new KineticState(
                motorShooter.getCurrentPosition(),
                motorShooter.getVelocity()
        )));
        actualVelocity = motorShooter.getVelocity();
        panelsTelemetry.addData("target vel", target);
        panelsTelemetry.addData("actual vel", actualVelocity);
        panelsTelemetry.update();


    }
}
