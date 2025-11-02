package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;

import java.util.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends LinearOpMode{
    Hardware robot = new Hardware();

    // State tracking
    private double spindexerTarget = 0.5; // Start at middle
    
    // Button X sequence state (turret + lift)
    private boolean buttonXSequenceActive = false;
    private final ElapsedTime buttonXSequenceTimer = new ElapsedTime();

    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode(){
        telemetry = robot.init(hardwareMap, telemetry);
        
        telemetry.update();
        waitForStart();

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

            //noinspection OptionalGetWithoutIsPresent
            Vector4 power = powerList.stream().reduce((a, b) -> add(a, b)).get().mult(SPEED_CONSTANT);
            //scale such that no motor is powered outside of [-1.0,1.0]
            power.div(max(1, power.stream().max().orElse(0) / SPEED_CONSTANT));
            power.mult(1 - 0.5 * gamepad1.right_trigger);
            robot.powerMotors(power);

            telemetry.addData("power w", power.w);
            telemetry.addData("power x", power.x);
            telemetry.addData("power y", power.y);
            telemetry.addData("power z", power.z);
            robot.logHeading();

            robot.logMotorPos();

            handleButtonControls();

            telemetry.addLine("--- Driver 2 Controls ---");
            telemetry.addData("GP2 RB (Reset)", gamepad2.right_bumper);
            telemetry.addData("GP2 LB (Turret+Lift)", gamepad2.left_bumper);
            telemetry.addData("GP2 A (140° CW)", gamepad2.a);
            telemetry.addData("GP2 B (60° CW)", gamepad2.b);
            telemetry.addData("GP2 Y (60° CCW)", gamepad2.y);
            telemetry.addData("GP2 X (135° CCW)", gamepad2.x);
            telemetry.addData("Spindexer Pos", String.format("%.3f", spindexerTarget));
            telemetry.addLine("");
            telemetry.addLine("--- Hardware Status ---");
            telemetry.addData("Spindexer", robot.spindexer != null ? "OK" : "NULL");
            telemetry.addData("Turret", robot.turretFlywheel != null ? "OK" : "NULL");
            telemetry.addData("Lift", robot.lift != null ? "OK" : "NULL");
            if (buttonXSequenceActive) {
                telemetry.addData("Turret+Lift Sequence", "Active - " + String.format("%.1f", buttonXSequenceTimer.seconds()) + "s");
            }
            
            telemetry.update();
        }

        robot.turretFlywheel.setPower(0);
        robot.lift.setPower(0);
    }
    
    private void handleButtonControls() {
        // Button RB: Reset spindexer to leftmost position (Driver 2)
        if (gamepad2.rightBumperWasPressed()) {
            spindexerTarget = SPINDEXER_LEFT_LIMIT;
            robot.spindexer.setPosition(spindexerTarget);
            telemetry.addLine("Button RB (GP2): Spindexer reset to left");
        }
        
        // Button B: 60 degrees clockwise (Driver 2)
        if (gamepad2.bWasPressed()) {
            spinSpindexerClockwise(60.0);
            telemetry.addLine("Button B (GP2): Spun 60° clockwise");
        }
        
        // Button X: 120 degrees counterclockwise (Driver 2)
        if (gamepad2.xWasPressed()) {
            spinSpindexerCounterclockwise(135.0);
            telemetry.addLine("Button X (GP2): Spun 120° counterclockwise");
        }
        
        // Button Y: 60 degrees counterclockwise (Driver 2)
        if (gamepad2.yWasPressed()) {
            spinSpindexerCounterclockwise(70.0);
            telemetry.addLine("Button Y (GP2): Spun 60° counterclockwise");
        }
        
        // Button A: 140 degrees clockwise (Driver 2)
        if (gamepad2.aWasPressed()) {
            spinSpindexerClockwise(140.0);
            telemetry.addLine("Button A (GP2): Spun 140° clockwise");

        }
        
        // Button LB: Turret + Lift sequence (Driver 2)
        if (gamepad2.leftBumperWasPressed() && !buttonXSequenceActive) {
            buttonXSequenceActive = true;
            buttonXSequenceTimer.reset();

            robot.turretFlywheel.setPower(1.0);
            telemetry.addLine("Button LB (GP2): Turret started at full power");


            robot.lift.setPower(1.0); // Start lifting up
            telemetry.addLine("Button LB (GP2): Lift started (up)");

            
            telemetry.addLine("Button LB (GP2): Turret+Lift sequence started");
        }
        
        // Handle turret + lift sequence timing
        handleTurretLiftSequence();
    }
    
    private void spinSpindexerCounterclockwise(double degrees) {
        // Counterclockwise (left) = moving toward LEFT_LIMIT (1.0 in this case)
        // Since LEFT_LIMIT (1.0) > RIGHT_LIMIT (0.0), counterclockwise = increasing position
        double positionChange = (degrees / SPINDEXER_DEGREE_RANGE) * Math.abs(SPINDEXER_LEFT_LIMIT - SPINDEXER_RIGHT_LIMIT);
        spindexerTarget = Math.min(SPINDEXER_LEFT_LIMIT, spindexerTarget + positionChange);
        robot.spindexer.setPosition(spindexerTarget);
    }
    
    private void spinSpindexerClockwise(double degrees) {
        // Clockwise (right) = moving toward RIGHT_LIMIT (0.0 in this case)
        // Since RIGHT_LIMIT (0.0) < LEFT_LIMIT (1.0), clockwise = decreasing position
        double positionChange = (degrees / SPINDEXER_DEGREE_RANGE) * Math.abs(SPINDEXER_LEFT_LIMIT - SPINDEXER_RIGHT_LIMIT);
        spindexerTarget = Math.max(SPINDEXER_RIGHT_LIMIT, spindexerTarget - positionChange);
        robot.spindexer.setPosition(spindexerTarget);
    }
    
    private void handleTurretLiftSequence() {
        if (!buttonXSequenceActive) return;

        double elapsedMs = buttonXSequenceTimer.seconds() * 1000.0;
        
        if (elapsedMs < LIFT_UP_DURATION_MS) {
            // Still lifting up (forward) - keep setting power to ensure it stays on
            robot.lift.setPower(1.0);
            // Keep turret running at full power
            robot.revTurret();
        } else if (elapsedMs < LIFT_UP_DURATION_MS + LIFT_DOWN_DURATION_MS) {
            // Switch to lifting down (reverse) - turret still running
            robot.lift.setPower(-1.0);
            robot.revTurret();

        } else {
            // Sequence complete - stop everything
            robot.lift.setPower(0);
            robot.endRevTurret();
            buttonXSequenceActive = false;
            telemetry.addLine("Turret+Lift sequence complete");
        }
    }
}