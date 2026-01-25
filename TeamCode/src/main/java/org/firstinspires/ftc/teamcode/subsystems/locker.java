package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class locker implements Subsystem {
    public static final locker INSTANCE = new locker();
    private final ServoEx locker = new ServoEx("locker");

    public Command open() {
        return new InstantCommand(new SetPosition(locker, 0.2)).requires(this);
    }
    public Command close() {
        return new InstantCommand(new SetPosition(locker, 0)).requires(this);
    }
    public double getPosition() {
        return locker.getPosition();
    }
    public void buttonMap() {
        Gamepads.gamepad1().square()
                .toggleOnBecomesTrue()
                .whenBecomesTrue(open())
                .whenBecomesFalse(close());

    }


}
