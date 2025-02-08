package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystem.DiddyArm;
import org.firstinspires.ftc.teamcode.Subsystem.DragonWarrior;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;

@Autonomous(name = "5+02.0")
public class A5_20 extends LinearOpMode {

    public double currentPivot, currentExt;
    DragonWarrior dragon = new DragonWarrior();
    Robot robot = new Robot();
    DiddyArm boxtube = new DiddyArm();
    MultipleTelemetry tele;

    public boolean conditionalEndPivot(double target) {

        currentPivot = boxtube.pivotoffset + (boxtube.Pivot.getCurrentPosition());
        currentPivot = -currentPivot;
        if (currentPivot >= target - 60 && currentPivot <= target + 60) {
            boxtube.Pivot.setPower(0);
            tele.addData("pivot end", true);
            return false;
        } else {
            tele.addData("pivot end", false);
            return true;
        }
    }

    public boolean conditionalEndExtension(double target) {
        currentExt = boxtube.BT1.getCurrentPosition();
        currentExt = -currentExt;
        if (currentExt >= target - 500 && currentExt <= target + 500) {
            boxtube.ExtensionPower(0);
            tele.addData("extension end", true);
            return false;
        } else {
            tele.addData("extension end", false);

            return true;
        }
    }

    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        dragon = new DragonWarrior();
         robot = new Robot();
         boxtube = new DiddyArm();

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
        Pose2d sample2Pose = new Pose2d(53, -11, Math.toRadians(90));
        Pose2d sample2PushPose = new Pose2d(53, -43, Math.toRadians(90));
        Pose2d sample3Pose = new Pose2d(58, -10, Math.toRadians(90));
        Pose2d sample3PushPose = new Pose2d(58, -47.5, Math.toRadians(90));
        Pose2d obsZonePoseClose = new Pose2d(35, -62, Math.toRadians(270));
        robot.LoiterAuto();


        TrajectoryActionBuilder Preload = drive.actionBuilder(intialPose).splineToLinearHeading(highChamberPose, Math.toRadians(90));
        TrajectoryActionBuilder Pushing = drive.actionBuilder(highChamberPose)
                .setTangent(Math.toRadians(0)).splineToLinearHeading(goingToSample1, Math.toRadians(90)).setTangent(Math.toRadians(90)).splineToLinearHeading(sample1Pose, Math.toRadians(0)).setTangent(Math.toRadians(270)).splineToLinearHeading(sample1PushPose, Math.toRadians(90)).setTangent(Math.toRadians(90)).splineToLinearHeading(sample2Pose, Math.toRadians(-45)).setTangent(Math.toRadians(270)).splineToLinearHeading(sample2PushPose, Math.toRadians(90)).setTangent(Math.toRadians(90)).splineToLinearHeading(sample3Pose, Math.toRadians(-45)).splineToLinearHeading(sample3PushPose, Math.toRadians(90))
                .setTangent(Math.toRadians(270)).splineToLinearHeading(sample1Pose, Math.toRadians(0))
                .splineToLinearHeading(sample1PushPose, Math.toRadians(90))
                .splineToLinearHeading(sample2Pose, Math.toRadians(0))
                .splineToLinearHeading(sample2PushPose, Math.toRadians(90))
                .splineToLinearHeading(sample3Pose, Math.toRadians(0))
                .splineToLinearHeading(sample3PushPose, Math.toRadians(90))
                ;

//        Action Pickup2Traj = drive.actionBuilder(sample3PushPose).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270)).build();
//        Action Score2Traj = drive.actionBuilder(obsZonePoseClose).setTangent(Math.toRadians(90)).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).build();

//        Action Pickup3Traj = drive.actionBuilder(highChamberPose2).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270)).build();
//        Action Score3Traj = drive.actionBuilder(obsZonePoseClose).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270)).build();
//
//        Action Pickup4Traj = drive.actionBuilder(highChamberPose2).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270)).build();
//        Action Score4Traj = drive.actionBuilder(obsZonePoseClose).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270)).build();
//
//        Action Pickup5Traj = drive.actionBuilder(highChamberPose2).splineToLinearHeading(obsZonePoseClose, Math.toRadians(270)).build();
//        Action Score5Traj = drive.actionBuilder(obsZonePoseClose).splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270)).build();

//        TrajectoryActionBuilder Scoring2Spec = drive.actionBuilder(sample3PushPose)
//                .splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
//                .splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270));
//
//        TrajectoryActionBuilder Scoring3Spec = drive.actionBuilder(highChamberPose2)
//                .splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
//                .splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270));
//
//        TrajectoryActionBuilder Scoring4Spec = drive.actionBuilder(sample3PushPose)
//                .splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
//                .splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270));
//        TrajectoryActionBuilder Scoring5Spec = drive.actionBuilder(sample3PushPose)
//                .splineToLinearHeading(obsZonePoseClose, Math.toRadians(270))
//                .splineToLinearHeading(highChamberPose2, Math.toRadians(90)).setTangent(Math.toRadians(270));



        Action PreloadTraj = Preload.build();
        Action PushTraj = Pushing.build();
//        Action ScoringTraj2 = Scoring2Spec.build();
//        Action ScoringTraj3 = Scoring3Spec.build();
//        Action ScoringTraj4 = Scoring4Spec.build();
//        Action ScoringTraj5 = Scoring5Spec.build();

        while (!isStopRequested() && !opModeIsActive()) {

        }

        waitForStart();

        if (isStopRequested()) return;


        Actions.runBlocking(new SequentialAction(
                        new ParallelAction(
                        PreloadTraj,
                        telemetryPacket -> {
                            tele.addLine("Pivot up ");
                            tele.addData("boxtube pos", boxtube.Pivot.getCurrentPosition());
                            tele.update();
                            boxtube.PivotMove(435);
                            return conditionalEndPivot(435);
                        },
                        new InstantAction(()-> robot.zero())
                        ),
                        telemetryPacket -> {
                            tele.addLine("Extend slides ");
                            tele.update();
                            boxtube.ExtensionMove(24500);
                            return conditionalEndExtension(24500);
                        },

                        telemetryPacket -> {
                            tele.addLine("Pivot down ");
                            tele.update();
                            boxtube.PivotMove(185);
                            return conditionalEndPivot(185);
                        },
                        new SleepAction(0.25),
                        new InstantAction(() -> robot.ClawOpen()),
                        telemetryPacket -> {
                            tele.addLine("Retract slides");
                            tele.update();
                            boxtube.ExtensionMove(2000);
                            return conditionalEndExtension(2000);
                        }
//                        new InstantAction(() -> robot.PushingSamplePos()),
//                        new ParallelAction(
//                                PushTraj,
//                                telemetryPacket -> {
//                                    tele.addLine("Pivot down ");
//                                    tele.update();
//                                    boxtube.PivotMove(0);
//                                    return conditionalEndPivot(0);
//                                }
//                                ),
//                        new SleepAction(0.5)
//                        new ParallelAction(
//                                Pickup2Traj,
//                                telemetryPacket -> {
//                                    tele.addLine("Pickup State");
//                                    tele.update();
//                                    robot.SpecimenWall();
//                                    return conditionalEndExtension(10000);
//                                }
//                        ),
//                        new InstantAction(() -> robot.ClawClose()),
//                telemetryPacket -> {
//                    tele.addLine("Pickup State");
//                    tele.update();
//                    robot.SpecimenWall();
//                    return conditionalEndExtension(10000);
//                },
//                        new ParallelAction(
//                                Score2Traj,
//                                new SequentialAction(
//                                telemetryPacket -> {
//                                    tele.addLine("Pickup State");
//                                    tele.update();
//                                    robot.SpecimenPreScore();
//                                    return conditionalEndPivot(700) && conditionalEndExtension(2000);
//                                },
//                                telemetryPacket -> {
//                                    tele.addLine("Pickup State");
//                                    tele.update();
//                                    robot.SpecimenLatch();
//                                    return conditionalEndExtension(20000);
//                                }
//
//                        )
                        )
        );;

    }
}
