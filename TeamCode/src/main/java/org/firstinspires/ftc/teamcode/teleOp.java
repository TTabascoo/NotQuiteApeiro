package org.firstinspires.ftc.teamcode;
import static dev.nextftc.extensions.pedro.PedroComponent.follower;
import com.pedropathing.geometry.BezierPoint;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.ftc.NextFTCOpMode;

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
//        shooter.INSTANCE.buttonMap(gamepad1);
        shooter.INSTANCE.testButtonMap(gamepad1);
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

//        if (gamepad1.shareWasPressed()) {
//            selfPath = follower().pathBuilder().addPath(new BezierPoint(follower().getPose().getX(), follower().getPose().getY())).build();
//            new FollowPath(selfPath, true, 1.0);
//        } //if this doesn't work uh oh

        telemetry.addData("left stick y", Gamepads.gamepad1().leftStickY().negate());
        telemetry.addData("left stick x", Gamepads.gamepad1().leftStickX());
        telemetry.addData("right stick x", Gamepads.gamepad1().rightStickX());
        telemetry.addData("fr power", driveTrain.INSTANCE.getFRPower());
        telemetry.addData("fl power", driveTrain.INSTANCE.getFLPower());
        telemetry.addData("br power", driveTrain.INSTANCE.getBRPower());
        telemetry.addData("bl power", driveTrain.INSTANCE.getBLPower());
        telemetry.addData("intake power", intake.INSTANCE.power());
        telemetry.addData("intake direction", intake.INSTANCE.getDirection());
        telemetry.addData("shooter power 1", shooter.INSTANCE.getPower1());
        telemetry.addData("shooter power 2", shooter.INSTANCE.getPower2());
        telemetry.addData("shooter direction", shooter.INSTANCE.getDirection());
        telemetry.addData("shooter goal", shooter.INSTANCE.getGoal());
        telemetry.addData("locker position", locker.INSTANCE.getPosition());
        telemetry.update();

    }
}
