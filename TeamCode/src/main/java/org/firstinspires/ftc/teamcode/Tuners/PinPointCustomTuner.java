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
@TeleOp(name = "PinPoint PD Tuner with Motion Profiling")
public class PinPointCustomTuner extends LinearOpMode {

    public static double TargetYPos = 0, TargetXPOS = 0, TargetHeading = 0;
    public static double kP_Y = 0.01, kP_X = 0.01, kP_H = 0.01; // Proportional Gains
    public static double kD_Y = 0.001, kD_X = 0.001, kD_H = 0.001; // Derivative Gains

    public static double maxVelocity = 20; // Max speed in inches/sec
    public static double maxAcceleration = 10; // Max acceleration in inches/sec²

    private double currentX = 0, currentY = 0, currentH = 0;
    private double prevErrorX = 0, prevErrorY = 0, prevErrorH = 0,prevVelY;

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
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU(); // Ensure robot is still for 0.25 seconds

        dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        // Reset the timer before the loop starts
        timer.reset();

        waitForStart();

        while (opModeIsActive()) {
            pose = odo.getPosition(); // Get X, Y, Heading

            // Get current position
            currentX = pose.getX(DistanceUnit.INCH);
            currentY = pose.getY(DistanceUnit.INCH);
            currentH = pose.getHeading(AngleUnit.DEGREES);

            // Calculate errors
            double errorX = TargetXPOS - currentX;
            double errorY = TargetYPos - currentY;
            double errorH = TargetHeading - currentH;

            // Calculate time difference
            double deltaTime = timer.seconds();
            timer.reset(); // Reset timer for the next loop

            // Compute derivative terms (rate of change of error)
            double derivativeX = (errorX - prevErrorX) / deltaTime;
            double derivativeY = (errorY - prevErrorY) / deltaTime;
            double derivativeH = (errorH - prevErrorH) / deltaTime;

            // Compute PD control outputs
            double xPower = (kP_X * errorX) + (kD_X * derivativeX);
            double yPower = (kP_Y * errorY) + (kD_Y * derivativeY);
            double hPower = (kP_H * errorH) + (kD_H * derivativeH);

            // Apply motion profiling to smooth velocity transitions
            xPower = motionProfile(TargetXPOS, currentX, maxVelocity, maxAcceleration);
            yPower = motionProfile(TargetYPos, currentY, maxVelocity, maxAcceleration);
            hPower = motionProfile(TargetHeading, currentH, maxVelocity, maxAcceleration);

            // Drive the robot
            drive.TeleopControl(-yPower, xPower, hPower); // Reverse Y stick value

            // Store previous errors for next loop iteration
            prevErrorX = errorX;
            prevErrorY = errorY;
            prevErrorH = errorH;

            // Send telemetry data
            tele.addData("Target X", TargetXPOS);
            tele.addData("Target Y", TargetYPos);
            tele.addData("Target H", TargetHeading);
            tele.addData("Current X", currentX);
            tele.addData("Current Y", currentY);
            tele.addData("Current H", currentH);
            tele.addData("Error X", errorX);
            tele.addData("Error Y", errorY);
            tele.addData("Error H", errorH);
            tele.addData("X Power (PD+MP)", xPower);
            tele.addData("Y Power (PD+MP)", yPower);
            tele.addData("H Power (PD+MP)", hPower);
            tele.addData("Loop Time (s)", deltaTime);
            double vel = odo.getVelY();
            tele.addData("Y vel", vel/25.4);
            tele.addData("Yaccel", (vel - prevVelY) / deltaTime);
            prevVelY = vel;

            tele.update();

            // Update odometry
            odo.update();

            sleep(10); // Prevent CPU overload
        }
    }

    /**
     * Motion profiling method for smooth velocity transitions.
     * Uses a trapezoidal velocity profile to limit acceleration and deceleration.
     *
     * @param target Target position
     * @param current Current position
     * @param maxVelocity Maximum velocity allowed
     * @param maxAccel Maximum acceleration allowed
     * @return Adjusted power for smooth motion
     */
    private double motionProfile(double target, double current, double maxVelocity, double maxAccel) {
        double error = target - current;
        double direction = Math.signum(error);

        // Compute desired velocity using a trapezoidal velocity profile
        double velocity = Math.sqrt(2 * maxAccel * Math.abs(error));
        velocity = Math.min(velocity, maxVelocity);

        // Convert velocity to motor power (Assumes power = velocity / maxVelocity)
        return (velocity / maxVelocity) * direction;
    }
}
