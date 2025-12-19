package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter () {}
    private MotorEx shooter = new MotorEx("shooter");
    private MotorEx shooter2 = new MotorEx("shooter2");
    public Command shoot(double seconds) {
        Timer CommandTimer = new Timer();
        return new LambdaCommand()
                .setStart(() -> {
                    CommandTimer.resetTimer();
                })
                .setUpdate(() -> {
                    new SetPower(shooter, 1);
                    new SetPower(shooter2, 1);
                })
                .setIsDone(() -> CommandTimer.getElapsedTimeSeconds() > seconds)
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

    public double power1() {
        return shooter.getPower();
    }
    public double power2() {
        return shooter2.getPower();
    }
}
