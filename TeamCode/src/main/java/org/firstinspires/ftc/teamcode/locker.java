package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class locker implements Subsystem {
    public static final locker INSTANCE = new locker();
    private final ServoEx locker = new ServoEx("locker");

    public Command open() {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPosition(locker, 1.0);
        }).setRequirements(this);
    }
    public Command close() {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPosition(locker, 0);
                }).setRequirements(this);
    }
    public double getPosition() {
        return locker.getPosition();
    }
    public void buttonMap(Gamepad gamepad) {
        Button open = button(() -> gamepad.square)
                .toggleOnBecomesTrue()
                .whenBecomesTrue(() -> open())
                .whenBecomesFalse(() -> close());

    }


}
