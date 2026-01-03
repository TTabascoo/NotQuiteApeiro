package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.shooterConstants.*;
import static dev.nextftc.bindings.Bindings.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter () {}
    private final MotorEx shooter = new MotorEx("shooter");
    private final MotorEx shooter2 = new MotorEx("shooter2");

    @Override
    public void initialize() {
        controller = ControlSystem.builder()
                .velPid(shooterConstants.coefficients)
                .basicFF(kV, kA, kS)
                .build();
        controller.setGoal(new KineticState(0, shootingSpotVel));
    }

    @Override
    public void periodic() {
        powerNeeded1 = controller.calculate(new KineticState(shooter.getCurrentPosition(), shooter.getVelocity()));
        powerNeeded2 = controller.calculate(new KineticState(shooter2.getCurrentPosition(), shooter2.getVelocity()));
    }

    public Command shoot(double power) {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPower(shooter, powerNeeded1);
                    new SetPower(shooter2, powerNeeded2);
                })
                .requires(this);
    }



    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPower(shooter, 0);
                    new SetPower(shooter2, 0);
                })
                .requires(this);
    }

    public double getPower1() {
        return shooter.getPower();
    }
    public double getPower2() { return shooter2.getPower();}

    public void switchDirections() {
        shooterdirection = shooterdirection*-1;
    }

    public void buttonMap(Gamepad gamepad) {
        Button toggleShooter = button(() -> gamepad.b)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> shoot(1*shooterdirection)) //TO DO CHANGE W PID
                .whenBecomesFalse(() -> stop());
        Button directionSwitch = button(() -> gamepad.dpadUpWasPressed())
                .whenBecomesTrue(() -> switchDirections());
    }

}
