package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.shooterConstants.target;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class tuningShooter extends NextFTCOpMode {
    private DcMotorEx motorShooter;
    private ControlSystem controller;

    @Override
    public void onInit() {
        motorShooter = hardwareMap.get(DcMotorEx.class, "shooter");
        controller = ControlSystem.builder()
                .velPid(shooterConstants.coefficients)
                .build();
        controller.setGoal(new KineticState(0, target, 0));
    }

    @Override
    public void onUpdate() {
        motorShooter.setPower(controller.calculate(new KineticState(
                motorShooter.getCurrentPosition(),
                motorShooter.getVelocity()
        )));
    }
}
