package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class teleOp extends OpMode {
    DcMotor intake;
    DcMotor shooter;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
    }

    @Override
    public void loop() {
        shooter.setPower(gamepad1.right_trigger);
        intake.setPower(gamepad1.left_trigger);

    }
}
