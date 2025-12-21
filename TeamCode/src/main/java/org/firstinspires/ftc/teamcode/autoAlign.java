package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.TurnBy;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@TeleOp
public class autoAlign extends NextFTCOpMode {
    Limelight3A limelight;
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void onInit() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "backRight");
    }

    @Override
    public void onUpdate() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx();
            double ty = result.getTy();
            double ta = result.getTa();
            Angle neededAngle = Angle.fromDeg(tx);

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.update();
            new SequentialGroup(  //im kinda scared bc this might just keep going-> it continually tries to fix itself and wouldnt let driver control happen
                    new TurnBy(neededAngle)
            );
        } else {
            telemetry.addData("Limelight", "No Targets");
            telemetry.update();
        }





    }
}
