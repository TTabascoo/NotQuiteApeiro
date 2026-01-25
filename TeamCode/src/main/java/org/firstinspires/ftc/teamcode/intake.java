package org.firstinspires.ftc.teamcode;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();

    private intake() {
    }

    private final MotorEx intake = new MotorEx("intake");

    public Command rampOn(double power) {
        return new InstantCommand(new SetPower(intake, power)).requires(this);
    }
    public Command rampOff() {
        return new InstantCommand(new SetPower(intake, 0)).requires(this);
    }

    public void buttonMap() {
        Gamepads.gamepad1().rightTrigger()
                .greaterThan(0.0)
                .whenBecomesTrue(rampOn(-0.8))
                .whenBecomesFalse(rampOff());

        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.2)
                .whenBecomesTrue(rampOn(0.9))
                .whenBecomesFalse(rampOff());
    }

}
