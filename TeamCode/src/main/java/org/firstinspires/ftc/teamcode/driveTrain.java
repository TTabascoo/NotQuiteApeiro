package org.firstinspires.ftc.teamcode;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class driveTrain implements Subsystem {
    public static final driveTrain INSTANCE = new driveTrain();
    private driveTrain() {
    }
    private final MotorEx frontLeft = new MotorEx( "frontLeft").reversed().brakeMode();
    private final MotorEx frontRight = new MotorEx("frontRight").reversed().brakeMode();
    private final MotorEx backLeft = new MotorEx( "backLeft").reversed().brakeMode();
    private final MotorEx backRight = new MotorEx("backRight").reversed().brakeMode();

    @Override
    public void initialize() {

    }
    public double getFLPower() {
        return frontLeft.getPower();
    }
    public double getFRPower() {
        return frontRight.getPower();
    }
    public double getBLPower() {
        return backLeft.getPower();
    }
    public double getBRPower() {
        return backRight.getPower();
    }
    //pov u read the documentation: i love built in commands wowie 67
    public Command driveControl() {
        return new MecanumDriverControlled(
                frontLeft,
                frontRight,
                backLeft,
                backRight,
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX()
                );

    }
}
