package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.intakeConstants.intakePower;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.variable;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Variable;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.intake;

@TeleOp
public class teleOp extends NextFTCOpMode {
    {
        addComponents(
                new SubsystemComponent(
                        intake.INSTANCE),
                new SubsystemComponent(
                        shooter.INSTANCE
                ),
                new SubsystemComponent(
                    locker.INSTANCE
                )
        );
    }
    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor backRight;
    Follower follower;
    PathChain selfPath;

    @Override
    public void onStartButtonPressed() {
        shooter.INSTANCE.buttonMap(gamepad1);
        intake.INSTANCE.buttonMap(gamepad1);
        locker.INSTANCE.buttonMap(gamepad1);
    }

    @Override
    public void onInit() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");
        follower = Constants.createFollower(hardwareMap);

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
        follower.update();
        BindingManager.update();

        if (gamepad1.shareWasPressed()) {
            selfPath = follower.pathBuilder().addPath(new BezierLine(follower.getPose(), follower.getPose())).build();
            new FollowPath(selfPath, true, 1.0);
        }



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
        telemetry.addData("intake power", intake.INSTANCE.power());
        telemetry.addData("shooter power", shooter.INSTANCE.getPower1());
        telemetry.addData("shooter power", shooter.INSTANCE.getPower2());
        telemetry.addData("intake power val ", intakePower);
        telemetry.addData("encoder FR", frontRight.getCurrentPosition());
        telemetry.addData("encoder FL", frontLeft.getCurrentPosition());
        telemetry.addData("encoder BR", backRight.getCurrentPosition());
        telemetry.addData("encoder BL", backLeft.getCurrentPosition());
        telemetry.update();

    }
}
