package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;

import java.util.*;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends LinearOpMode{
    Hardware robot = new Hardware();

    public static final Vector4 VDrive  = of(+1d, +1d, +1d, +1d);
    public static final Vector4 VStrafe = of(+1d, -1d, -1d, +1d);
    public static final Vector4 VTurn   = of(+1d, -1d, +1d, -1d);
    
    // Spindexer and other hardware
    private Servo spindexer;
    private DcMotor turret;
    private CRServo lift;
    
    // Spindexer configuration
    // Swapped: 0.0 appears to be rightmost on your servo, so 1.0 = leftmost
    private static final double SPINDEXER_LEFT_LIMIT = 1.0;   // Leftmost position for intake
    private static final double SPINDEXER_RIGHT_LIMIT = 0;  // Rightmost position
    private static final double SPINDEXER_DEGREE_RANGE = 300.0; // Total servo range in degrees
    
    // State tracking
    private double currentSpindexerPosition = 0.5; // Start at middle
    
    // Button state tracking
    private boolean lastRB = false;
    private boolean lastA = false;
    private boolean lastB = false;
    private boolean lastY = false;
    private boolean lastX = false;
    private boolean lastLB = false;  // Left bumper for turret+lift sequence
    
    // Button X sequence state (turret + lift)
    private boolean buttonXSequenceActive = false;
    private com.qualcomm.robotcore.util.ElapsedTime buttonXSequenceTimer = new com.qualcomm.robotcore.util.ElapsedTime();
    private static final long LIFT_UP_DURATION_MS = 3500;
    private static final long LIFT_DOWN_DURATION_MS = 2000;

    @Override
    public void runOpMode(){
        telemetry = robot.init(hardwareMap, telemetry);
        
        // Initialize spindexer servo
        try {
            spindexer = hardwareMap.get(Servo.class, "spindexer");
            spindexer.setPosition(currentSpindexerPosition);
            telemetry.addLine("Spindexer initialized");
        } catch (Exception e) {
            spindexer = null;
            telemetry.addLine("WARNING: Spindexer not found!");
        }
        
        // Initialize turret motor (called "turretFly")
        try {
            turret = hardwareMap.get(DcMotor.class, "turretFly");
            turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            telemetry.addLine("Turret (turretFly) initialized");
        } catch (Exception e) {
            turret = null;
            telemetry.addLine("WARNING: Turret motor 'turretFly' not found!");
            telemetry.addLine("Configure 'turretFly' as DcMotor in Robot Configuration");
        }
        
        // Initialize lift servo (called "lift", CRServo)
        try {
            lift = hardwareMap.get(CRServo.class, "lift");
            telemetry.addLine("Lift (lift) initialized as CRServo");
        } catch (Exception e) {
            lift = null;
            telemetry.addLine("WARNING: Lift servo 'lift' not found!");
            telemetry.addLine("Configure 'lift' as CRServo in Robot Configuration");
        }
        
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

            robot.logMotorPos();
            
            // Handle button controls (Driver 2)
            handleButtonControls();
            
            // Debug: Show button states
            telemetry.addLine("--- Driver 2 Controls ---");
            telemetry.addData("GP2 RB (Reset)", gamepad2.right_bumper);
            telemetry.addData("GP2 LB (Turret+Lift)", gamepad2.left_bumper);
            telemetry.addData("GP2 A (140° CW)", gamepad2.a);
            telemetry.addData("GP2 B (60° CW)", gamepad2.b);
            telemetry.addData("GP2 Y (60° CCW)", gamepad2.y);
            telemetry.addData("GP2 X (135° CCW)", gamepad2.x);
            telemetry.addData("Spindexer Pos", String.format("%.3f", currentSpindexerPosition));
            telemetry.addLine("");
            telemetry.addLine("--- Hardware Status ---");
            telemetry.addData("Spindexer", spindexer != null ? "OK" : "NULL");
            telemetry.addData("Turret", turret != null ? "OK" : "NULL");
            telemetry.addData("Lift", lift != null ? "OK" : "NULL");
            if (buttonXSequenceActive) {
                telemetry.addData("Turret+Lift Sequence", "Active - " + String.format("%.1f", buttonXSequenceTimer.seconds()) + "s");
            }
            
            telemetry.update();
        }
        
        // Stop all on exit
        if (turret != null) turret.setPower(0);
        if (lift != null) lift.setPower(0);
    }
    
    private void handleButtonControls() {
        // Button RB: Reset spindexer to leftmost position (Driver 2)
        if (gamepad2.right_bumper && !lastRB) {
            if (spindexer != null) {
                currentSpindexerPosition = SPINDEXER_LEFT_LIMIT;
                spindexer.setPosition(currentSpindexerPosition);
                telemetry.addLine("Button RB (GP2): Spindexer reset to left");
            } else {
                telemetry.addLine("ERROR: Spindexer is null! Check Robot Configuration.");
            }
        }
        lastRB = gamepad2.right_bumper;
        
        // Button B: 60 degrees clockwise (Driver 2)
        if (gamepad2.b && !lastB) {
            if (spindexer != null) {
                spinSpindexerClockwise(60.0);
                telemetry.addLine("Button B (GP2): Spun 60° clockwise");
            } else {
                telemetry.addLine("ERROR: Spindexer is null!");
            }
        }
        lastB = gamepad2.b;
        
        // Button X: 120 degrees counterclockwise (Driver 2)
        if (gamepad2.x && !lastX) {
            if (spindexer != null) {
                spinSpindexerCounterclockwise(135.0);
                telemetry.addLine("Button X (GP2): Spun 120° counterclockwise");
            } else {
                telemetry.addLine("ERROR: Spindexer is null!");
            }
        }
        lastX = gamepad2.x;
        
        // Button Y: 60 degrees counterclockwise (Driver 2)
        if (gamepad2.y && !lastY) {
            if (spindexer != null) {
                spinSpindexerCounterclockwise(70.0);
                telemetry.addLine("Button Y (GP2): Spun 60° counterclockwise");
            } else {
                telemetry.addLine("ERROR: Spindexer is null!");
            }
        }
        lastY = gamepad2.y;
        
        // Button A: 140 degrees clockwise (Driver 2)
        if (gamepad2.a && !lastA) {
            if (spindexer != null) {
                spinSpindexerClockwise(140.0);
                telemetry.addLine("Button A (GP2): Spun 140° clockwise");
            } else {
                telemetry.addLine("ERROR: Spindexer is null!");
            }
        }
        lastA = gamepad2.a;
        
        // Button LB: Turret + Lift sequence (Driver 2)
        if (gamepad2.left_bumper && !lastLB && !buttonXSequenceActive) {
            buttonXSequenceActive = true;
            buttonXSequenceTimer.reset();
            
            if (turret != null) {
                turret.setPower(1.0);
                telemetry.addLine("Button LB (GP2): Turret started at full power");
            } else {
                telemetry.addLine("ERROR: Turret motor is null!");
            }
            
            if (lift != null) {
                lift.setPower(1.0); // Start lifting up
                telemetry.addLine("Button LB (GP2): Lift started (up)");
            } else {
                telemetry.addLine("ERROR: Lift servo is null!");
            }
            
            telemetry.addLine("Button LB (GP2): Turret+Lift sequence started");
        }
        lastLB = gamepad2.left_bumper;
        
        // Handle turret + lift sequence timing
        handleTurretLiftSequence();
    }
    
    private void spinSpindexerCounterclockwise(double degrees) {
        // Counterclockwise (left) = moving toward LEFT_LIMIT (1.0 in this case)
        // Since LEFT_LIMIT (1.0) > RIGHT_LIMIT (0.0), counterclockwise = increasing position
        double positionChange = (degrees / SPINDEXER_DEGREE_RANGE) * Math.abs(SPINDEXER_LEFT_LIMIT - SPINDEXER_RIGHT_LIMIT);
        currentSpindexerPosition = Math.min(SPINDEXER_LEFT_LIMIT, currentSpindexerPosition + positionChange);
        spindexer.setPosition(currentSpindexerPosition);
    }
    
    private void spinSpindexerClockwise(double degrees) {
        // Clockwise (right) = moving toward RIGHT_LIMIT (0.0 in this case)
        // Since RIGHT_LIMIT (0.0) < LEFT_LIMIT (1.0), clockwise = decreasing position
        double positionChange = (degrees / SPINDEXER_DEGREE_RANGE) * Math.abs(SPINDEXER_LEFT_LIMIT - SPINDEXER_RIGHT_LIMIT);
        currentSpindexerPosition = Math.max(SPINDEXER_RIGHT_LIMIT, currentSpindexerPosition - positionChange);
        spindexer.setPosition(currentSpindexerPosition);
    }
    
    private void handleTurretLiftSequence() {
        if (!buttonXSequenceActive) return;
        
        double elapsedSeconds = buttonXSequenceTimer.seconds();
        double elapsedMs = elapsedSeconds * 1000.0;
        
        if (elapsedMs < LIFT_UP_DURATION_MS) {
            // Still lifting up (forward) - keep setting power to ensure it stays on
            if (lift != null) {
                lift.setPower(1.0);
            }
            // Keep turret running at full power
            if (turret != null) {
                turret.setPower(1.0);
            }
        } else if (elapsedMs < LIFT_UP_DURATION_MS + LIFT_DOWN_DURATION_MS) {
            // Switch to lifting down (reverse) - turret still running
            if (lift != null) {
                lift.setPower(-1.0);
            }
            if (turret != null) {
                turret.setPower(1.0);
            }
        } else {
            // Sequence complete - stop everything
            if (lift != null) {
                lift.setPower(0);
            }
            if (turret != null) {
                turret.setPower(0);
            }
            buttonXSequenceActive = false;
            telemetry.addLine("Turret+Lift sequence complete");
        }
    }
}