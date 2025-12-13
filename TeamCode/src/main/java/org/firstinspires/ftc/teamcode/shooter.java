package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class shooter implements Subsystem {
    public static final shooter INSTANCE = new shooter();
    private shooter () {}
    private MotorEx shooter = new MotorEx("shooter");
    public Command shoot = new SetPower(shooter, 1).requires(this);
    public double power() {
        return shooter.getPower();
    }

}
