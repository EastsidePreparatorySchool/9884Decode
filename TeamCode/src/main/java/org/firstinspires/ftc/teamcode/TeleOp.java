package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import android.annotation.SuppressLint;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.lib.Coroutine;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;

import java.util.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends OpMode{
    Hardware robot = new Hardware();
    Coroutine.Manager manager = new Coroutine.Manager();

    private int spindexerTarget = SpindexerPosition.A_IN;

    @Override
    public void init(){
        telemetry = robot.init(hardwareMap, telemetry);
        telemetry.update();

        manager.start(new DriveTrainRoutine());
        manager.start(new SpindexerRoutine());

        telemetry.addLine("--- Driver 2 Controls ---");
        telemetry.addData("GP2 RB (Reset)", gamepad2.right_bumper);
        telemetry.addData("GP2 LB (Turret+Lift)", gamepad2.left_bumper);
        telemetry.addData("GP2 A (140° CW)", gamepad2.a);
        telemetry.addData("GP2 B (60° CW)", gamepad2.b);
        telemetry.addData("GP2 Y (60° CCW)", gamepad2.y);
        telemetry.addData("GP2 X (135° CCW)", gamepad2.x);
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void loop(){
        manager.loop(getRuntime());
        telemetry.update();
    }
    @Override
    public void stop(){
        robot.turretFlywheel.setPower(0);
        robot.lift.setPower(0);
    }

    private class SpindexerRoutine extends Coroutine{
        private int state = 0;

        @Nullable
        @Override
        protected Object loop(){
            final boolean increment = gamepad2.leftBumperWasPressed();

            telemetry.addData("Spindexer Pos", spindexerTarget);
            switch (state){
                case 0:
                    spindexerTarget = SpindexerPosition.A_IN;
                    robot.setSpindexer(spindexerTarget);
                    if (increment) state = 1;
                    return null;
                case 1:
                    spindexerTarget = SpindexerPosition.B_IN;
                    robot.setSpindexer(spindexerTarget);
                    if (increment) state = 2;
                    return null;
                case 2:
                    spindexerTarget = SpindexerPosition.C_IN;
                    robot.setSpindexer(spindexerTarget);
                    if (increment) state = 3;
                    return Coroutine.Yield.delay(SPINDEXER_ROTATION_TIME);
                case 3:
                    robot.revTurret();
                    spindexerTarget = SpindexerPosition.B_OUT;
                    robot.setSpindexer(spindexerTarget);
                    robot.lift.setPower(1.0);
                    state = 4;
                    return Coroutine.Yield.delay(LIFT_UP_TIME);
                case 4:
                    robot.lift.setPower(-1);
                    state = 5;
                    return Coroutine.Yield.delay(LIFT_DOWN_TIME);
                case 5:
                    robot.lift.setPower(0);
                    robot.setSpindexer(SpindexerPosition.A_OUT);
                    state = 6;
                    return Coroutine.Yield.delay(SPINDEXER_ROTATION_TIME);
                case 6:
                    robot.lift.setPower(1.0);
                    state = 7;
                    return Coroutine.Yield.delay(LIFT_UP_TIME);
                case 7:
                    robot.lift.setPower(-1);
                    state = 8;
                    return Coroutine.Yield.delay(LIFT_DOWN_TIME);
                case 8:
                    robot.lift.setPower(0);
                    robot.setSpindexer(SpindexerPosition.C_OUT);
                    state = 9;
                    return Coroutine.Yield.delay(SPINDEXER_ROTATION_TIME);
                case 9:
                    robot.lift.setPower(1.0);
                    state = 10;
                    return Coroutine.Yield.delay(LIFT_UP_TIME);
                case 10:
                    robot.lift.setPower(-1);
                    state = 11;
                    return Coroutine.Yield.delay(LIFT_DOWN_TIME);
                case 11:
                    robot.lift.setPower(0);
                    state = 0;
                    robot.endRevTurret();
                    return null;
            }

            throw new IllegalStateException();
        }
    }

    private class DriveTrainRoutine extends Coroutine{
        @Nullable
        @Override
        protected Object loop(){
            double lx = gamepad1.left_stick_x;
            double ly = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;

            telemetry.addData("lx", lx);
            telemetry.addData("ly", ly);
            telemetry.addData("rx", rx);

            telemetry.addLine(VDrive.toString());
            telemetry.addLine(mult(VDrive, 1).toString());

            ArrayList<Vector4> powerList = new ArrayList<>();
            powerList.add(mult(VDrive, ly));
            powerList.add(mult(VStrafe, lx));
            powerList.add(mult(VTurn, rx));

            Vector4 power = powerList.stream().reduce((a, b) -> add(a, b)).get().mult(SPEED_CONSTANT);
            //scale such that no motor is powered outside of [-1.0,1.0]
            power.div(max(1, power.stream().max().orElse(0) / SPEED_CONSTANT));
            power.mult(1 - 0.5 * gamepad1.right_trigger);
            robot.powerMotors(power);

            telemetry.addData("power w", power.w);
            telemetry.addData("power x", power.x);
            telemetry.addData("power y", power.y);
            telemetry.addData("power z", power.z);

            robot.logMotorPos();
            robot.logHeading();

            return null;
        }
    }
}