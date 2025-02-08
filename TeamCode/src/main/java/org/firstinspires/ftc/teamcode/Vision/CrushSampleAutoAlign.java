package org.firstinspires.ftc.teamcode.Vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Crush Sample Auto Align")
public class CrushSampleAutoAlign extends LinearOpMode{
    private CrushSampleAnglePipeline pipeline;
    private VisionPortal VP;
    private FtcDashboard dash;
    private MultipleTelemetry tele;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the pipeline
        pipeline = new CrushSampleAnglePipeline();
        dash = FtcDashboard.getInstance();
        dash.getInstance().startCameraStream(pipeline,0);
        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Angle", pipeline.getDetectedAngle());
            telemetry.update();
        }
    }

}
