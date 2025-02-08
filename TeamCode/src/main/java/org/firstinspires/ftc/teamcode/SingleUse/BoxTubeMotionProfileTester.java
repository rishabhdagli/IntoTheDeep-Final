package org.firstinspires.ftc.teamcode.SingleUse;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class BoxTubeMotionProfileTester extends LinearOpMode {

    public DcMotorEx slides1, slides2, slides3;
    MultipleTelemetry tele;

    // Motion Profile constants
    public static double maxAcceleration = 5.0; // m/s^2
    public static double maxVelocity = 10.0;    // m/s
    public static double distance = 50.0;        // meters (convert to encoder ticks based on your robot's configuration)

    // Constants for Proportional control
    public static double kP = 0.1;

    // Time step for profiling
    public static double timeStep = 0.1;
    public double elapsedTime;
    public static int targetPosition = 1000;

    @Override
    public void runOpMode() throws InterruptedException {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        slides1 = hardwareMap.get(DcMotorEx.class, "slides1");
        slides2 = hardwareMap.get(DcMotorEx.class, "slides2");
        slides3 = hardwareMap.get(DcMotorEx.class, "slides3");

        slides1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides3.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slides1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slides3.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        slides1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slides3.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while (opModeIsActive() ) {
            int currentPosition = slides1.getCurrentPosition();
            double pos = motionProfile(maxAcceleration, maxVelocity, distance, elapsedTime);
            double error = pos - currentPosition;
            double output = kP * error;
            slides1.setPower(output);
            slides2.setPower(output);
            slides3.setPower(output);


            tele.addData("Desired Position:", pos);
            tele.addData("Current Position:", currentPosition);
            tele.update();

            sleep(100);
        }
    }

    // Function to calculate the motion profile for a single motor (based on max acceleration, max velocity, and distance)
    public double motionProfile(double maxAcceleration, double maxVelocity, double distance, double elapsedTime) {
        // Calculate the time it takes to accelerate to max velocity
        double accelerationTime = maxVelocity / maxAcceleration;

        // Calculate distance covered during acceleration (constant acceleration)
        double accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);

        // If we can't accelerate to max velocity in the given distance, adjust the acceleration time
        if (accelerationDistance > distance / 2) {
            accelerationTime = Math.sqrt(distance / (2 * maxAcceleration));
            accelerationDistance = 0.5 * maxAcceleration * Math.pow(accelerationTime, 2);
        }

        // Calculate the distance to cruise at max velocity
        double cruiseDistance = distance - 2 * accelerationDistance;
        double cruiseTime = cruiseDistance / maxVelocity;

        // Time to decelerate (same as acceleration)
        double decelerationTime = accelerationTime;

        // Calculate the total time for the motion profile
        double totalTime = accelerationTime + cruiseTime + decelerationTime;

        // If we have passed the total time, return the final position (end of profile)
        if (elapsedTime > totalTime) {
            return distance;
        }

        // Accelerating phase
        if (elapsedTime < accelerationTime) {
            return 0.5 * maxAcceleration * Math.pow(elapsedTime, 2);
        }

        // Cruising phase
        else if (elapsedTime < accelerationTime + cruiseTime) {
            return accelerationDistance + maxVelocity * (elapsedTime - accelerationTime);
        }

        // Decelerating phase
        else {
            double decelerationElapsedTime = elapsedTime - (accelerationTime + cruiseTime);
            return distance - 0.5 * maxAcceleration * Math.pow(decelerationElapsedTime, 2);
        }
    }
}
