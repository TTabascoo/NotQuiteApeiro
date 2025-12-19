package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();

    private intake() {
    }

    private MotorEx intake = new MotorEx("intake");

    public double power() {
        return intake.getPower();
    }

    public Command upRamp(double seconds) {
        Timer CommandTimer = new Timer();
        return new LambdaCommand()
                .setStart(() -> {
                    CommandTimer.resetTimer();
                })
                .setUpdate(() -> {
                    new SetPower(intake, 1);
                })
                .setIsDone(() -> CommandTimer.getElapsedTimeSeconds() > seconds)
                .requires(this);
    }


    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPower(intake, 0);
                })
                .requires(this);
    }
}
