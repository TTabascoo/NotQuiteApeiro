package org.firstinspires.ftc.teamcode;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();
    private intake () {}
    private MotorEx intake = new MotorEx("intake");
    public Command upRamp = new SetPower(intake, 1).requires(this);
    public double power() {
        return intake.getPower();
    }
}
