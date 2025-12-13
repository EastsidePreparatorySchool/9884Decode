package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class ServoTest extends OpMode{
    public static String name = "";

    Servo servo;

    public static double position = 0;

    @Override
    public void init(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        servo = hardwareMap.servo.get(name);
        servo.getController().pwmEnable();
    }

    @Override
    public void loop(){
        servo.setPosition(position);
        telemetry.addData(name, servo.getPosition());
        telemetry.addData("ctrl" + name, servo.getController().getServoPosition(servo.getPortNumber()));
        telemetry.update();
    }
}
