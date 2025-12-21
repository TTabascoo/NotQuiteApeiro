package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class teleOp extends NextFTCOpMode {
    DcMotor intake;
    DcMotor shooter;
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    double intakePower;
    double shooterPower;
    double shooterdirection = 1;
    double shootermultiplier = 0;
    double intakemultiplier = 0;
    double intakedirection = 1;
    int shooterCounter = 1;
    int intakeCounter = 1;

    @Override
    public void onInit() {
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);




    }

    @Override
    public void onUpdate() {
        shooterPower = shooterdirection*gamepad1.right_trigger;
        intakePower = intakedirection * intakemultiplier;
        
        if (gamepad1.dpadDownWasPressed()) {
            intakedirection = intakedirection*-1;
        }
        if (gamepad1.dpadUpWasPressed()) {
            shooterdirection = shooterdirection*-1;
        }


        if (gamepad1.bWasPressed()) {
            intakeCounter = intakeCounter + 1;
        }
        if (intakeCounter % 2 ==0 ) {
            intakemultiplier = 1;
        } else {
            intakemultiplier = 0;
        }

        intake.setPower(intakePower);
        shooter.setPower(shooterPower);

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        double frontLeftPower = (y + x + rx);
        double backLeftPower = (y - x + rx) ;
        double frontRightPower = (y - x - rx);
        double backRightPower = (y + x - rx);

        frontLeft.setPower(frontLeftPower);
        frontRight.setPower(frontRightPower);
        backLeft.setPower(backLeftPower);
        backRight.setPower(backRightPower);

        telemetry.addData("value of gamepad x", x);
        telemetry.addData("value of gamepad y", y);
        telemetry.addData("value of gamepad rx", rx);
        telemetry.addData("intake counter", intakeCounter);
        telemetry.addData("shooter counter", shooterCounter);
        telemetry.addData("intake power", intake.getPower());
        telemetry.addData("shooter power", shooter.getPower());
        telemetry.addData("intake power val ", intakePower);
        telemetry.addData("shooter power val", shooterPower);
        telemetry.addData("encoder FR", frontRight.getCurrentPosition());
        telemetry.addData("encoder FL", frontLeft.getCurrentPosition());
        telemetry.addData("encoder BR", backRight.getCurrentPosition());
        telemetry.addData("encoder BL", backLeft.getCurrentPosition());
        telemetry.update();

    }
}
