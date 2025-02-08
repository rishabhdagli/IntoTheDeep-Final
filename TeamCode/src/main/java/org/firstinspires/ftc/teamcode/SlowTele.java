package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.LED;
import com.sfdev.assembly.state.StateMachine;

import org.firstinspires.ftc.teamcode.Subsystem.DiddyArm;
import org.firstinspires.ftc.teamcode.Subsystem.DragonWarrior;
import org.firstinspires.ftc.teamcode.Subsystem.Robot;
import org.firstinspires.ftc.teamcode.Subsystem.StateMachineGenerator;


// This teleop is a drive only 50% speed. Made for Science-Fair
@TeleOp(name = "Drive-Only Slow TeleOp (50% Speed)")
public class SlowTele extends LinearOpMode {

    public DragonWarrior dragon;

    @Override
    public void runOpMode() throws InterruptedException {
        dragon = new DragonWarrior();
        dragon.init(hardwareMap);


        waitForStart();
        while (opModeIsActive()) {
            dragon.TeleopControl(gamepad1.left_stick_y * 0.5, gamepad1.left_stick_x * 0.5, gamepad1.right_stick_x / 2.0);
        }
    }
}