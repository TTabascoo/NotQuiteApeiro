package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.teamcode.intakeConstants.intakedirection;
import static dev.nextftc.bindings.Bindings.*;
import static dev.nextftc.bindings.Bindings.variable;
import com.qualcomm.robotcore.hardware.Gamepad;
import dev.nextftc.bindings.Button;
import dev.nextftc.bindings.Variable;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
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

    public Command autoRamp(double power) {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPower(intake, power);

                })
                .requires(this);
    }

    public void switchDirections() {
        intakedirection = intakedirection*-1;
    }
    public double getDirection() {
        return intakedirection;
    }

    public Command stop() {
        return new LambdaCommand()
                .setStart(() -> {
                    new SetPower(intake, 0);
                })
                .requires(this);
    }
    public void buttonMap(Gamepad gamepad) {
        Variable<Float> itrigger = variable(() -> gamepad.right_trigger);
        Button intakeButton = itrigger.asButton(value -> value>0);
        intakeButton.whenTrue(autoRamp(gamepad.right_trigger*intakedirection));

        Button intakeSwitch = button(() -> gamepad.dpadDownWasPressed())
                .whenBecomesTrue(() -> switchDirections());

    }

}
