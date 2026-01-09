package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.shooterConstants.*;
import static dev.nextftc.bindings.Bindings.*;
import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter () {}
    private final MotorEx shooter = new MotorEx("shooter");
    private final MotorEx shooter2 = new MotorEx("shooter2");
    public MotorGroup shooterGroup = new MotorGroup(shooter, shooter2);

    @Override
    public void initialize() {
        shooter.zeroed();
        shooter2.zeroed();
        controller = ControlSystem.builder()
                .velPid(shooterConstants.coefficients)
                .basicFF(kV, kA, kS)
                .build();
    }


    // LOOK AT NEXT FTC DOCS: PERIODIC RUNS EVERY LOOP AS LONG AS U INCORPORATE SUBSYSTEM
    // The commands shot and stop simply create new goals for the motors, which will be run every loop
    @Override
    public void periodic() {
//        shooterGroup.setPower(controller.calculate(shooterGroup.getState()));
//        ActiveOpMode.telemetry().addData("shooter1 power", shooter.getPower());
//        ActiveOpMode.telemetry().addData("shooter2 power", shooter2.getPower());
//        ActiveOpMode.telemetry().addData("direction", shooterdirection);
//        ActiveOpMode.telemetry().update();
    }

    public LambdaCommand shoot(double direction) {
        return new LambdaCommand()
                .setStart(() -> {
                    new RunToVelocity(controller, shootingSpotVel*direction);
                })
                .requires(this);
    }



    public LambdaCommand stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    new RunToVelocity(controller, 0);
                })
                .requires(this);
    }
    //NON PIDF shooter commands for testing


    public Command testShoot() {
        return new InstantCommand(new SetPower(shooterGroup, 1).requires(this));
    }
    public Command testStop() {
        return new InstantCommand(new SetPower(shooterGroup, 0).requires(this));
    }


    public double getPower1() {
        return shooter.getPower();
    }
    public double getPower2() { return shooter2.getPower();}

    public void switchDirections() {
        shooterdirection = shooterdirection*-1;
    }

    public double getDirection() {
        return shooterdirection;
    }

    public double getGoal() {
        return controller.calculate(shooterGroup.getState());
    }
    public void buttonMap() {
        Gamepads.gamepad1().b()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(testShoot())
                .whenBecomesFalse(testStop());
        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> switchDirections());
    }
    public boolean reachedTarget() {
        return Math.abs(shooter.getVelocity() - target) <= threshold;
    }

}
