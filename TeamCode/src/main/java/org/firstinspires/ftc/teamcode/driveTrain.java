package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class driveTrain implements Subsystem {
    public static final driveTrain INSTANCE = new driveTrain();
    private driveTrain() {
    }
    private MotorEx frontLeft = new MotorEx( "frontLeft").reversed().brakeMode();
    private MotorEx frontRight = new MotorEx("frontRight").reversed().brakeMode();
    private MotorEx backLeft = new MotorEx( "backLeft").reversed().brakeMode();
    private MotorEx backRight = new MotorEx("backRight").reversed().brakeMode();
    private MotorGroup dTMotors = new MotorGroup(frontLeft, frontRight, backLeft, backRight);

    @Override
    public void initialize() {

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
