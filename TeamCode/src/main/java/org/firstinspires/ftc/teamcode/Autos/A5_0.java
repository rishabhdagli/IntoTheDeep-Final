package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.DiddyArm;
import org.firstinspires.ftc.teamcode.Subsystem.DragonWarrior;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@Autonomous(name = "5+0")
public class A5_0 extends LinearOpMode {

    DragonWarrior dragon = new DragonWarrior();
    Robot robot = new Robot();
    DiddyArm boxtube = new DiddyArm();
    MultipleTelemetry tele;
    public double currentPivot, currentExt;


    public boolean conditionalEndPivot(double target) {
        currentPivot = boxtube.pivotoffset + (boxtube.Pivot.getCurrentPosition());
        currentPivot = -currentPivot;
        if (currentPivot >= target - 25 && currentPivot <= target + 25) {
            boxtube.Pivot.setPower(0);
            return false;
        } else {
            return true;
        }
    }

    public boolean conditionalEndExtension(double target) {
        currentExt = boxtube.BT1.getCurrentPosition();
        currentExt = -currentExt;
        if (currentExt >= target - 500 && currentExt <= target + 500) {
            boxtube.ExtensionPower(0);
            return false;
        } else {
            return true;
        }
    }


    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        dragon.init(hardwareMap);
        robot.init(hardwareMap, gamepad2);
        boxtube.init(hardwareMap);


        Pose2d intialPose = new Pose2d(17, -62, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, intialPose);
        Pose2d highChamberPose = new Pose2d(5, -52, Math.toRadians(90));
        Pose2d highChamberPose2 = new Pose2d(7, -42, Math.toRadians(270));
        Pose2d goingToSample1 = new Pose2d(36 , -34, Math.toRadians(90));
        Pose2d sample1Pose = new Pose2d(46, -10, Math.toRadians(90));
        Pose2d sample1PushPose = new Pose2d(46, -43, Math.toRadians(90));
        Pose2d sample2Pose = new Pose2d(53, -8, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(53, -43, Math.toRadians(90));
        Pose2d sample3Pose = new Pose2d(58, -5, Math.toRadians(90));
        Pose2d sample3PushPose = new Pose2d(58, -47.5, Math.toRadians(90));
        Pose2d obsZonePoseClose = new Pose2d(35, -62, Math.toRadians(270));


        TrajectoryActionBuilder Preload = drive.actionBuilder(intialPose)
                .splineToLinearHeading(highChamberPose, Math.toRadians(90))
                .setTangent(Math.toRadians(0)).splineToLinearHeading(goingToSample1, Math.toRadians(90))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(sample1Pose, Math.toRadians(0))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(sample1PushPose, Math.toRadians(90))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(sample2Pose, Math.toRadians(-45))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(sample2PushPose, Math.toRadians(90))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(sample3Pose, Math.toRadians(-45))
                .splineToLinearHeading(sample3PushPose, Math.toRadians(90))
                .splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
                .setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90));
        Action path1 = Preload.build();

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;

        Actions.runBlocking(path1);
    }
}
