package org.firstinspires.ftc.teamcode.RoadRunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Config
@Autonomous(name="Trajectory Test")
public class TrajectoryTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(48, 48, Math.toRadians(-90)));

        Action myTrajectory = drive.actionBuilder(drive.pose)
                .lineToY(45)
                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), 0)
                .splineToLinearHeading(new Pose2d(58, 45, Math.toRadians(-90)), 0)
                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), 90)
                .splineToLinearHeading(new Pose2d(56, 45, Math.toRadians(300)), -90)
                .splineToLinearHeading(new Pose2d(58, 55, Math.toRadians(225)), 0)
                .splineToLinearHeading(new Pose2d(23, 20, 0), 30)
                .build();
        waitForStart();

        if(isStopRequested()) return;

        Actions.runBlocking(new SequentialAction(myTrajectory));
    }
    //                        .lineToY(24)
//                        .waitSeconds(2.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-48)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-58)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(-90))
//                        .lineToX(-68)
//                        .turn(Math.toRadians(90))
//                        .lineToY(33)
//                        .waitSeconds(1.0)
//                        .lineToY(40)
//                        .turn(Math.toRadians(90))
//                        .lineToX(55)
//                        .turn(Math.toRadians(-90))
//                        .lineToY(63)
    }

