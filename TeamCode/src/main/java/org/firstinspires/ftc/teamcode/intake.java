package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.intakeConstants.intakedirection;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.impl.MotorEx;
import dev.nextftc.hardware.powerable.SetPower;

public class intake implements Subsystem {
    public static final intake INSTANCE = new intake();

    private intake() {
    }

    private final MotorEx intake = new MotorEx("intake");

    public double power() {
        return intake.getPower();
    }

//    public LambdaCommand autoRamp(double power) {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    new SetPower(intake, power);
//                })
////                .setInterruptible(true)
//                .requires(this);
//    }

    public Command rampOn(double power) {
        return new InstantCommand( new SetPower(intake, power)).requires(this);
    }
    public Command rampOff() {
        return new InstantCommand(new SetPower(intake, 0)).requires(this);
    }



    public void switchDirections() {
        intakedirection = intakedirection*-1;
    }
    public double getDirection() {
        return intakedirection;
    }

//    public LambdaCommand stop() {
//        return new LambdaCommand()
//                .setStart(() -> {
//                    new SetPower(intake, 0);
//                })
//                .requires(this);
//    }
    public void buttonMap() {
        Gamepads.gamepad1().rightTrigger()
                .greaterThan(0.0)
                .whenBecomesTrue(rampOn(1*intakedirection))
                .whenBecomesFalse(rampOff());

        Gamepads.gamepad1().leftTrigger()
                .greaterThan(0.0)
                .whenBecomesTrue(rampOn(-1*intakedirection))
                .whenBecomesFalse(rampOff());

        Gamepads.gamepad1().dpadDown()
                .whenBecomesTrue(this::switchDirections);
    }

}
