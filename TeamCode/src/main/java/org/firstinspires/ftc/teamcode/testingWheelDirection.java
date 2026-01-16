package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.ftc.NextFTCOpMode;


@TeleOp
public class testingWheelDirection extends NextFTCOpMode {
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    double y;
    @Override
    public void onInit() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void onUpdate() {
        if(gamepad1.triangleWasPressed()) {
            frontLeft.setPower(1);
        }
        if(gamepad1.squareWasPressed()) {
            frontRight.setPower(1);
        }
        if(gamepad1.circleWasPressed()) {
            backLeft.setPower(1);
        }
        if(gamepad1.crossWasPressed()) {
            backRight.setPower(1);
        }
        y = -gamepad1.left_stick_y;
        frontLeft.setPower(y);
        backLeft.setPower(y);
        frontRight.setPower(y);
        backRight.setPower(y);

    }
}
