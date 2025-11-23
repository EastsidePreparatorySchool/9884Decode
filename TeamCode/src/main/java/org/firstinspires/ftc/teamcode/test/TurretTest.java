package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.lib.Hardware;

@TeleOp
@Config
public class TurretTest extends LinearOpMode{
    Hardware robot;
    public static double turretPower;
    public static double spindexerPosition;
    public static double liftPower;

    @Override
    public void runOpMode() throws InterruptedException{
        telemetry = robot.init(hardwareMap, telemetry);
        waitForStart();
        while(opModeIsActive()){
            //telemetry.addData("liftpos", robot.getLiftPos());
            robot.setSpindexer(spindexerPosition);
            //robot.lift.setPower(liftPower);
            robot.turretFlywheel.setPower(turretPower);
        }
    }
}
