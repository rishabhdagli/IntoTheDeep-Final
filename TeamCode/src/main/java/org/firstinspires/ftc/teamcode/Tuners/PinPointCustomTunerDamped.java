package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Pinpoint.GoBildaPinpointDriver;
import org.firstinspires.ftc.teamcode.Subsystem.DragonWarrior;

@Config
@TeleOp(name = "PinPoint PD Tuner")
public class PinPointCustomTunerDamped extends LinearOpMode {

    public static double TargetYPos = 0, TargetXPOS = 0, TargetHeading = 0;
    public static double kP_Y = 0, kP_X = 0, kP_H = 0; // Proportional Gains
    public static double kD_Y = 0, kD_X = 0, kD_H = 0; // Derivative Gains

    private double currentX = 0, currentY = 0, currentH = 0;
    private double prevErrorX = 0, prevErrorY = 0, prevErrorH = 0;

    private ElapsedTime timer = new ElapsedTime();

    private GoBildaPinpointDriver odo;
    private Pose2D pose;
    private DragonWarrior drive;
    private MultipleTelemetry tele;
    private FtcDashboard dashboard;

    @Override
    public void runOpMode() throws InterruptedException {

        drive = new DragonWarrior();
        drive.init(hardwareMap);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.setOffsets(83.96967, 144.662); // in mm (X offset for perpendicular, Y offset for parallel)
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU(); // Ensure robot is still for 0.25 seconds

        dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        timer.reset();

        waitForStart();

        while (opModeIsActive()) {
            pose = odo.getPosition(); // Get current X, Y, and heading

            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);
            currentH = pose.getHeading(AngleUnit.DEGREES);

            // Compute errors directly from final target positions (field frame)
            double errorX = TargetXPOS - currentX;
            double errorY = TargetYPos - currentY;
            double errorH = TargetHeading - currentH;

            double deltaTime = timer.seconds();
            timer.reset(); // Reset timer for next loop iteration

            // Compute derivative terms (rate of change of error)
            double derivativeX = (errorX - prevErrorX) / deltaTime;
            double derivativeY = (errorY - prevErrorY) / deltaTime;
            double derivativeH = (errorH - prevErrorH) / deltaTime;

            // Compute PD control outputs in the field frame
            double xPowerField = (kP_X * errorX) + (kD_X * derivativeX);
            double yPowerField = (kP_Y * errorY) + (kD_Y * derivativeY);
            double hPower = (kP_H * errorH) + (kD_H * derivativeH);

            // Convert field coordinate outputs to robot-centric outputs for mecanum drive
            // The current heading is converted from degrees to radians.
            double headingRadians = currentH * (Math.PI / 180.0);
            double cosHeading = Math.cos(headingRadians);
            double sinHeading = Math.sin(headingRadians);

            // Transformation equations:
            // forward  = xPowerField * cos(heading) + yPowerField * sin(heading)
            // strafe   = -xPowerField * sin(heading) + yPowerField * cos(heading)
            double forward = (xPowerField * cosHeading) + (yPowerField * sinHeading);
            double strafe  = (-xPowerField * sinHeading) + (yPowerField * cosHeading);

            // Drive the robot using the mecanum drive control method.
            // Here, the "forward" component is negated if your robot configuration requires it.
            drive.TeleopControl(-forward, strafe, hPower);

            // Update previous errors for the next iteration
            prevErrorX = errorX;
            prevErrorY = errorY;
            prevErrorH = errorH;

            // Send telemetry data for debugging
            tele.addData("Target X", TargetXPOS);
            tele.addData("Target Y", TargetYPos);
            tele.addData("Target H", TargetHeading);
            tele.addData("Current X", currentX);
            tele.addData("Current Y", currentY);
            tele.addData("Current H", currentH);
            tele.addData("Error X", errorX);
            tele.addData("Error Y", errorY);
            tele.addData("Error H", errorH);
            tele.addData("X Power Field", xPowerField);
            tele.addData("Y Power Field", yPowerField);
            tele.addData("Forward (Robot)", forward);
            tele.addData("Strafe (Robot)", strafe);
            tele.addData("Heading Power (PD)", hPower);
            tele.addData("Loop Time (s)", deltaTime);
            tele.update();

            // Update odometry
            odo.update();
        }
    }
}
