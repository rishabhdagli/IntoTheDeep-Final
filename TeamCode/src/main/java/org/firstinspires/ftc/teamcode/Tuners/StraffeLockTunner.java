package org.firstinspires.ftc.teamcode.Tuners;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Subsystem.DragonWarrior;

@Config
@TeleOp(name = "Straffe lock tuner")
public class StraffeLockTunner extends LinearOpMode {

    //Make sure to chnage this in the dragon opmode
    public IMU Rotism;
    public double angle;

    public static double kpRotate;

    public DragonWarrior drive;
    MultipleTelemetry tele;
    FtcDashboard dashboard;



    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        tele = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new DragonWarrior();
        drive.init(hardwareMap);

        Rotism = hardwareMap.get(IMU.class, "imu");
        Rotism.initialize(new IMU.Parameters
                (new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP))
        ); //init imu end
        Rotism.resetYaw();
        angle = Rotism.getRobotYawPitchRollAngles().getYaw();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("melon botics beatingm", angle);
            if(gamepad1.right_stick_x == 0){
                //The error would be Rotism.getYaw();
                angle = Rotism.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                double error = 0 - angle;
                drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,kpRotate*error);
            }
            else {
                Rotism.resetYaw();
                drive.TeleopControl(gamepad1.left_stick_y,gamepad1.left_stick_x,gamepad2.right_stick_x);
            }
            telemetry.update();
        }
    }


}
