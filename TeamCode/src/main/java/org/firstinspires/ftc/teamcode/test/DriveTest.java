package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@Autonomous
@Config
public class DriveTest extends LinearOpMode{
    Hardware robot = new Hardware();

    @Override
    public void runOpMode(){
        telemetry = robot.init(hardwareMap, telemetry);
        waitForStart();

        for (DcMotor m : robot.driveMotors) {
            telemetry.addData("Motor", hardwareMap.getNamesOf(m).stream().findFirst().get());
            telemetry.update();
            m.setPower(1);
            robot.sleep(1000);
            m.setPower(0);
        }
    }
}
