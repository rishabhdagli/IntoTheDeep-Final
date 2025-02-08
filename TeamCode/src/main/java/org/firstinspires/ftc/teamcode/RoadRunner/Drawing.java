package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public final class Drawing {
    private Drawing() {}


    public static void drawRobot(Canvas c, Pose2d t) {
        final double ROBOT_RADIUS = 9;

        c.setStrokeWidth(1);
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS);

        Vector2d halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS);
        Vector2d p1 = t.position.plus(halfv);
        Vector2d p2 = p1.plus(halfv);
        c.strokeLine(p1.x, p1.y, p2.x, p2.y);
    }

    public static class LocalizationTest extends LinearOpMode {
        @Override
        public void runOpMode() throws InterruptedException {
            telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

            if (Localizer.TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
                MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();

                while (opModeIsActive()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ),
                            -gamepad1.right_stick_x
                    ));

                    drive.updatePoseEstimate();

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                    telemetry.update();

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().setStroke("#3F51B5");
                    drawRobot(packet.fieldOverlay(), drive.pose);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }
            } else if (Localizer.TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
                TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

                waitForStart();

                while (opModeIsActive()) {
                    drive.setDrivePowers(new PoseVelocity2d(
                            new Vector2d(
                                    -gamepad1.left_stick_y,
                                    0.0
                            ),
                            -gamepad1.right_stick_x
                    ));

                    drive.updatePoseEstimate();

                    telemetry.addData("x", drive.pose.position.x);
                    telemetry.addData("y", drive.pose.position.y);
                    telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
                    telemetry.update();

                    TelemetryPacket packet = new TelemetryPacket();
                    packet.fieldOverlay().setStroke("#3F51B5");
                    drawRobot(packet.fieldOverlay(), drive.pose);
                    FtcDashboard.getInstance().sendTelemetryPacket(packet);
                }
            } else {
                throw new RuntimeException();
            }
        }
    }
}
