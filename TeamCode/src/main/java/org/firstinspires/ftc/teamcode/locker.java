package org.firstinspires.ftc.teamcode;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.bindings.Button;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;

public class locker implements Subsystem {
    public static final locker INSTANCE = new locker();
    private final ServoEx locker = new ServoEx("locker");

    @Override
    public void periodic() {
//        ActiveOpMode.telemetry().addData("locker position", locker.getPosition());
//        ActiveOpMode.telemetry().update();
    }

//    public LambdaCommand open() {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    new SetPosition(locker, 1.0);
//        }).setRequirements(this);
//    }
//    public LambdaCommand close() {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    new SetPosition(locker, 0);
//                }).setRequirements(this);
//    }
    public Command open() {
        return new InstantCommand(new SetPosition(locker, 1.0)).requires(this);
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
