package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.intakeConstants.intakePower;

import static dev.nextftc.bindings.Bindings.button;
import static dev.nextftc.bindings.Bindings.variable;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;

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
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;
import org.firstinspires.ftc.teamcode.intake;

@TeleOp
public class teleOp extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                new SubsystemComponent(intake.INSTANCE),
                new SubsystemComponent(shooter.INSTANCE),
                new SubsystemComponent(locker.INSTANCE),
                new SubsystemComponent(driveTrain.INSTANCE)
        );
    }
    PathChain selfPath;

    @Override
    public void onStartButtonPressed() {
        shooter.INSTANCE.buttonMap(gamepad1);
        intake.INSTANCE.buttonMap(gamepad1);
        locker.INSTANCE.buttonMap(gamepad1);
        driveTrain.INSTANCE.driveControl().schedule();

    }

    @Override
    public void onInit() {
    }

    @Override
    public void onUpdate() {
        follower().update();
        BindingManager.update();

        if (gamepad1.shareWasPressed()) {
            selfPath = follower().pathBuilder().addPath(new BezierLine(follower().getPose(), follower().getPose())).build();
            new FollowPath(selfPath, true, 1.0);
        } //if this doesnt work uh oh

        telemetry.addData("left stick y", Gamepads.gamepad1().leftStickY().negate());
        telemetry.addData("left stick x", Gamepads.gamepad1().leftStickX());
        telemetry.addData("right stick x", Gamepads.gamepad1().rightStickX());
        telemetry.addData("intake power", intake.INSTANCE.power());
        telemetry.addData("shooter power", shooter.INSTANCE.getPower1());
        telemetry.addData("shooter power", shooter.INSTANCE.getPower2());
        telemetry.update();

    }
}
