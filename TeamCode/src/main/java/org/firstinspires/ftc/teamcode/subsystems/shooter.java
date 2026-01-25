package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.constants.shooterConstants.*;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter () {}
    private final MotorEx shooter = new MotorEx("shooter");
    private final MotorEx shooter2 = new MotorEx("shooter2");
    private ControlSystem controller;
    public MotorGroup shooterGroup = new MotorGroup(shooter, shooter2);

    @Override
    public void initialize() {
        shooter.zeroed();
        shooter2.zeroed();
        controller = ControlSystem.builder()
                .velPid(coefficients)
                .basicFF(ffcoefficients)
                .build();
        controller.setGoal(new KineticState(0, 0));
    }


    // LOOK AT NEXT FTC DOCS: PERIODIC RUNS EVERY LOOP AS LONG AS U INCORPORATE SUBSYSTEM
    // The commands shot and stop simply create new goals for the motors, which will be run every loop
    @Override
    public void periodic() {
        double powerNeeded = controller.calculate(new KineticState(
                shooter.getCurrentPosition() ,
                shooter.getVelocity()
        ));
        shooter.setPower(powerNeeded);
        shooter2.setPower(-powerNeeded);
    }

    public void shoot() {
        controller.setGoal(new KineticState(0, target));
    }
    public void stop() {
        controller.setGoal(new KineticState(0, 0));
    }
    public Command stopCommand2() {
        return new InstantCommand(() -> controller.setGoal(new KineticState(0, 0)));
    }

    public Command shootCommand() {
        return new RunToVelocity(controller, target, tolerance).setInterruptible(true).requires(this);
    }
    public Command stopCommand() {
        return new RunToVelocity(controller, target, tolerance).setInterruptible(true).requires(this);
    }

    public double getPower1() {
        return shooter.getPower();
    }
    public double getPower2() { return shooter2.getPower();}
    public double getVelocity() {
        return shooter.getVelocity();
    }
    public double getTarget() {
        return controller.getGoal().getVelocity();
    }

    public void switchDirections() {
        shooterdirection = shooterdirection*-1;
    }

    public void setTarget(double newTarget) {
        target = newTarget;
    }
    public void buttonMap() {
        Gamepads.gamepad1().b()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> shoot())
                .whenBecomesFalse(() -> stop());

        Gamepads.gamepad1().dpadUp()
                .whenBecomesTrue(() -> switchDirections());

        Gamepads.gamepad2().share()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> setTarget(1650))
                .whenBecomesFalse(() -> setTarget(2050));
    }
    public boolean reachedTarget() {
        return Math.abs(shooter.getVelocity() - target) <= threshold;
    }

}
