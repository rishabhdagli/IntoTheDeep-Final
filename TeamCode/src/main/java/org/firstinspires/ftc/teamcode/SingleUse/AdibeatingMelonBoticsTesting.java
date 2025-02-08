package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Absolute encoder testing")
@Config
public class AdibeatingMelonBoticsTesting extends LinearOpMode {

    // Get analog port instance from hardwareMap
    @Override
    public void runOpMode() throws InterruptedException {
        AnalogInput PivotAbs = hardwareMap.get(AnalogInput.class, "pivotAbs");
        AnalogInput BTAbs = hardwareMap.get(AnalogInput.class, "boxtubeAbs");
        DcMotorEx BT1 = hardwareMap.get(DcMotorEx.class, "Boxtube1ENC");
        DcMotorEx Pivot = hardwareMap.get(DcMotorEx.class, "pivotENC");

        BT1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        BT1.setDirection(DcMotorSimple.Direction.REVERSE);

        BT1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        double v = 0;
                //-1267.85714*(PivotAbs.getVoltage()) + 472.91071;
        double w = 0;
                //1241.11283*(BTAbs.getVoltage()) - 3749.40185;

        while (opModeIsActive()) {
            double position = PivotAbs.getVoltage() / 3.2 * 360;

            double currentPivot = -Pivot.getCurrentPosition();
            double currentBoxtube = -BT1.getCurrentPosition();

            double absolutepivot = v+ currentPivot;
            double absoluteBoxutbe = w + currentBoxtube;





            telemetry.addData("Pivot voltage", PivotAbs.getVoltage());
            telemetry.addData("pivot ticks raw", currentPivot);
            telemetry.addData("Pivot with absolute", absolutepivot);

            telemetry.addData("Boxtube Voltage", BTAbs.getVoltage());
            telemetry.addData("Boxtube ticks raw", currentBoxtube);
            telemetry.addData("Boxtube ticks Absolute",absoluteBoxutbe);
            telemetry.update();
        }
    }
}
