package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class teleOp extends OpMode {
    DcMotor intake;
    DcMotor shooter;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    double intakePower;
    double shooterPower;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        shooterPower = gamepad1.right_trigger;
        intakePower = gamepad1.left_trigger;
        
        if (gamepad1.dpadDownWasPressed()) {
            shooterPower = shooterPower*-1;
        }
        if (gamepad1.dpadUpWasPressed()) {
            intakePower = intakePower*-1;
        }

        shooter.setPower(shooterPower);
        intake.setPower(intakePower);

        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = -y + x + rx;
        double frontRightPower = y + x + rx;
        double backLeftPower = -y - x + rx;
        double backRightPower = y - x + rx;

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("value of gamepad x", x);
        telemetry.addData("value of gamepad y", y);
        telemetry.addData("value of gamepad rx", rx);
        telemetry.addData("fl power", frontLeft.getPower());
        telemetry.addData("fr power", frontRight.getPower());
        telemetry.addData("bl power", backLeft.getPower());
        telemetry.addData("br power", backRight.getPower());
        telemetry.addData("intake power", intake.getPower());
        telemetry.addData("shooter power", shooter.getPower());
        telemetry.addData("intake power val ", intakePower);
        telemetry.addData("shooter power val", shooterPower);
        telemetry.update();






    }
}
