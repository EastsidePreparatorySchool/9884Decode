package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@Autonomous(name = "AutoRight", group = "9884")
@Disabled
public class AutoRight extends LinearOpMode{
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        telemetry = robot.init(hardwareMap, telemetry);

        waitForStart();

//        robot.moveTime(Hardware.VStrafe, Hardware.AUTO_CONSTANT, 1000);
//        robot.moveTime(Hardware.VDrive, Hardware.AUTO_CONSTANT, 10_000);
    }
}
