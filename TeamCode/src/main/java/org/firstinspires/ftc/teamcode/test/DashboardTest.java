package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
@Disabled
public class DashboardTest extends LinearOpMode{
    public static int jork = 0;

    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Initialization Status Successful");
        telemetry.addLine("Transmission Interval:" + telemetry.getMsTransmissionInterval());
        telemetry.update();

        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("jork", jork);
            telemetry.update();
        }
    }
}
