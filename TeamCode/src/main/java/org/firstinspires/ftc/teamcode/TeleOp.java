package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;

import java.util.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends LinearOpMode{
    Hardware robot = new Hardware();

    public static final Vector4 VDrive  = of(+1d, +1d, +1d, +1d);
    public static final Vector4 VStrafe = of(+1d, -1d, -1d, +1d);
    public static final Vector4 VTurn   = of(+1d, -1d, +1d, -1d);

    @Override
    public void runOpMode(){
        telemetry = robot.init(hardwareMap, telemetry);
        waitForStart();

        boolean aPressed = false;
        int clawPos = 0;
        int sprocketPos = robot.sprocket.getCurrentPosition();

        while(opModeIsActive()){
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            telemetry.addData("lx", lx);
            telemetry.addData("ly", ly);
            telemetry.addData("rx", rx);

            System.out.printf(Locale.ENGLISH, "%.2f %.2f %.2f", lx, ly, rx);

            telemetry.addLine(VDrive.toString());
            telemetry.addLine(mult(VDrive, 1).toString());

            ArrayList<Vector4> powerList = new ArrayList<>();
            powerList.add(mult(VDrive, ly));
            powerList.add(mult(VStrafe, lx));
            powerList.add(mult(VTurn, rx));
            for(Vector4 i : powerList)
                telemetry.addLine(i.toString());

            //noinspection OptionalGetWithoutIsPresent
            Vector4 power = powerList.stream().reduce((a, b) -> add(a, b)).get().mult(SPEED_CONSTANT);
            double max = max(1, power.stream().max().orElse(0) / SPEED_CONSTANT);
            power.div(max);
            power.mult(1 - 0.5 * gamepad1.right_trigger);
            robot.powerMotors(power);

            telemetry.addData("powers", power.toString());
            telemetry.addData("power w", power.w);
            telemetry.addData("power x", power.x);
            telemetry.addData("power y", power.y);
            telemetry.addData("power z", power.z);
            robot.logHeading();

            int actuatorpos = robot.actuator.getCurrentPosition();
            if(gamepad2.dpad_up && actuatorpos > ACTUATOR_MAX_POS + ACTUATOR_SAFETY_BUFFER && robot.sprocket.getCurrentPosition() > -1350)
                robot.actuator.setPower(actuatorpos > ACTUATOR_MAX_POS + 2 * ACTUATOR_SAFETY_BUFFER ? 1 : 0.5);
            else if(gamepad2.dpad_down && actuatorpos < -ACTUATOR_SAFETY_BUFFER)
                robot.actuator.setPower(actuatorpos < 2 * -ACTUATOR_SAFETY_BUFFER ? -1 : -0.5);
            else robot.actuator.setPower(0);

            if(gamepad2.a) sprocketPos = SPROCKET_REST;
            if(gamepad2.b) sprocketPos = SPROCKET_WALL;
            //if(gamepad2.x) sprocketPos = SPROCKET_BAR;

            sprocketPos += Math.signum(gamepad2.right_stick_y) * SPROCKET_CONSTANT * (gamepad2.left_bumper ? SPROCKET_SLOW_MULT : 1);
            sprocketPos = Math.max(sprocketPos, -1600);
            sprocketPos = Math.min(sprocketPos, 0);
            robot.sprocket.setTargetPosition(sprocketPos);

            if(gamepad2.right_bumper && !aPressed){
                aPressed = true;
                clawPos = 1 - clawPos;
                robot.claw.setPosition(clawPos / 1.5d);
            }
            if(!gamepad2.right_bumper) aPressed = false;

            robot.logMotorPos();
            robot.logPose();
            telemetry.update();
        }
    }
}