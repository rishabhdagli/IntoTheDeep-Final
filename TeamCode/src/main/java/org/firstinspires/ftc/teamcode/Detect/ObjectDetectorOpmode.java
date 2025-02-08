package org.firstinspires.ftc.teamcode.Detect;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@TeleOp(name="Vision Tunning")
public class ObjectDetectorOpmode extends LinearOpMode {
    private ObejectDectectionPipeline pipeline;
    private VisionPortal VP;

    private FtcDashboard dash;

    //user interface
    public static int state = 0;
    //Case 0: Maked functionality
    public static int hueMin = 0,satMin = 100 ,valMin = 0,hueMax = 30,satMax = 255,valMax = 255;


    //Case 1: Object detection Functionality


    //Standard Blur is applied.
    // Every increase by 1 in blurIncrement will increase teh kernal size by an odd number ensuring no crashes
    //Number can be Negative as well. General Range: (no blur)-5 < 0 < 5 (a lot of blur) Can go further but not reccomended
    
     public static int blurIncrement = 0;

     //Canny is held Standard. Implementing not reccomended
    //public static int Canny = 0;




    public static double Angle = 60;

    @Override
    public void runOpMode() throws InterruptedException {



        // Initialize the pipeline
        pipeline = new ObejectDectectionPipeline();

        dash.getInstance().startCameraStream(pipeline,0);

        // Create the VisionPortal with the pipeline
        VP = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), pipeline);

        waitForStart();
        // Main loop during OpMode
        while (opModeIsActive()) {


        }





    }//run opmode end
}//class end
