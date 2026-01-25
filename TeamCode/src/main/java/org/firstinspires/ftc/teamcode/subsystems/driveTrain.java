package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.Direction;
import dev.nextftc.hardware.impl.IMUEx;
import dev.nextftc.hardware.impl.MotorEx;

public class driveTrain implements Subsystem {
    public static final driveTrain INSTANCE = new driveTrain();
    private driveTrain() {
    }
    public final MotorEx frontLeft = new MotorEx( "frontLeft").brakeMode().reversed();
    public final MotorEx frontRight = new MotorEx("frontRight").brakeMode();
    public final MotorEx backLeft = new MotorEx( "backLeft").brakeMode().reversed();
    public final MotorEx backRight = new MotorEx("backRight").brakeMode();
    public IMUEx imu = new IMUEx("imu", Direction.BACKWARD, Direction.UP).zeroed();



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
    public DriverControlledCommand driveControl() {
        return new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(), //MAYBE NEGATE
                Gamepads.gamepad1().leftStickX(),
                Gamepads.gamepad1().rightStickX(),
                false
        );
    }
    public DriverControlledCommand driveControl2 = new PedroDriverControlled(
            Gamepads.gamepad1().leftStickY().negate(), //MAYBE NEGATE
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate().mapToRange(place -> 0.8*place),
                true
                        );



}
