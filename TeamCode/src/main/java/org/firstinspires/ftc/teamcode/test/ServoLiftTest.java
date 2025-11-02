package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Dashboard test for Lift servo (servo0)
 * 
 * Configure in Dashboard:
 * - Move lift one direction (forward/up)
 * - Move lift other direction (reverse/down)
 * - Set duration for each direction
 * - Trigger tests via dashboard
 */
@Autonomous(name = "Servo Lift Test", group = "Test")
@Config
public class ServoLiftTest extends LinearOpMode {
    
    // Servo Configuration
    public static String servoName = "servo0";  // Lift servo name
    public static boolean isContinuous = true;  // true for CRServo, false for Servo
    
    // Direction 1 (Forward/Up) Configuration
    public static double direction1Power = 1.0;  // For CRServo: -1.0 to 1.0, For Servo: 0.0 to 1.0
    public static double direction1Duration = 2.0;  // Duration in seconds
    public static boolean runDirection1 = false;  // Set to true to run direction 1
    
    // Direction 2 (Reverse/Down) Configuration
    public static double direction2Power = -1.0;  // For CRServo: -1.0 to 1.0, For Servo: 0.0 to 1.0
    public static double direction2Duration = 2.0;  // Duration in seconds
    public static boolean runDirection2 = false;  // Set to true to run direction 2
    
    // Timing variables
    private static boolean lastRunDirection1 = false;
    private static boolean lastRunDirection2 = false;
    
    private ElapsedTime servoTimer = new ElapsedTime();
    private boolean servoRunning = false;
    private boolean currentDirection = true;  // true = direction1, false = direction2
    
    private CRServo crServo;
    private Servo servo;
    
    @Override
    public void runOpMode() {
        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
            telemetry, 
            FtcDashboard.getInstance().getTelemetry()
        );
        telemetry.setMsTransmissionInterval(50);
        
        telemetry.addLine("Servo Lift Test");
        telemetry.addLine("Configure in Dashboard at: 192.168.43.1:8080/dash");
        telemetry.addLine("");
        telemetry.addLine("Instructions:");
        telemetry.addLine("1. Set direction1Power/duration for one direction");
        telemetry.addLine("2. Set direction2Power/duration for other direction");
        telemetry.addLine("3. Set isContinuous (true for CRServo, false for Servo)");
        telemetry.addLine("4. Set runDirection1 or runDirection2 to true to trigger");
        telemetry.update();
        
        waitForStart();
        
        // Initialize servo
        initializeServo();
        
        while (opModeIsActive()) {
            // Check for direction 1 trigger
            if (runDirection1 && !lastRunDirection1 && !servoRunning) {
                startServo(true);
                lastRunDirection1 = true;
            } else if (!runDirection1) {
                lastRunDirection1 = false;
            }
            
            // Check for direction 2 trigger
            if (runDirection2 && !lastRunDirection2 && !servoRunning) {
                startServo(false);
                lastRunDirection2 = true;
            } else if (!runDirection2) {
                lastRunDirection2 = false;
            }
            
            // Update running servo
            updateServo();
            
            // Display status
            displayStatus();
            
            telemetry.update();
            sleep(50);
        }
        
        // Stop servo on exit
        stopServo();
    }
    
    private void initializeServo() {
        try {
            if (isContinuous) {
                crServo = hardwareMap.get(CRServo.class, servoName);
                telemetry.addLine("Lift servo (CRServo) initialized: '" + servoName + "'");
            } else {
                servo = hardwareMap.get(Servo.class, servoName);
                telemetry.addLine("Lift servo (Servo) initialized: '" + servoName + "'");
            }
        } catch (Exception e) {
            telemetry.addLine("ERROR: Lift servo '" + servoName + "' not found!");
            telemetry.addLine("Configure '" + servoName + "' as " + (isContinuous ? "CRServo" : "Servo") + " in Robot Configuration");
        }
    }
    
    private void startServo(boolean direction1) {
        currentDirection = direction1;
        double power = direction1 ? direction1Power : direction2Power;
        double duration = direction1 ? direction1Duration : direction2Duration;
        
        if (isContinuous && crServo != null) {
            crServo.setPower(power);
            servoTimer.reset();
            servoRunning = true;
        } else if (!isContinuous && servo != null) {
            servo.setPosition(power);
            servoTimer.reset();
            servoRunning = true;
        }
    }
    
    private void updateServo() {
        if (servoRunning) {
            double duration = currentDirection ? direction1Duration : direction2Duration;
            
            if (servoTimer.seconds() >= duration) {
                if (isContinuous && crServo != null) {
                    crServo.setPower(0);
                }
                servoRunning = false;
                
                // Auto-reset triggers
                if (currentDirection) {
                    runDirection1 = false;
                } else {
                    runDirection2 = false;
                }
            }
        }
    }
    
    private void displayStatus() {
        telemetry.clearAll();
        telemetry.addLine("=== Servo Lift Test ===");
        telemetry.addLine("Servo: " + servoName + " (" + (isContinuous ? "CRServo" : "Servo") + ")");
        telemetry.addLine("");
        
        if (servoRunning) {
            double duration = currentDirection ? direction1Duration : direction2Duration;
            double power = currentDirection ? direction1Power : direction2Power;
            String directionName = currentDirection ? "Direction 1" : "Direction 2";
            
            telemetry.addLine("Status: RUNNING - " + directionName);
            telemetry.addData("Elapsed", String.format("%.2f", servoTimer.seconds()) + "s / " + duration + "s");
            telemetry.addData("Power", power);
        } else {
            telemetry.addLine("Status: IDLE");
        }
        
        telemetry.addLine("");
        telemetry.addLine("=== Direction 1 Settings ===");
        telemetry.addData("Power", direction1Power);
        telemetry.addData("Duration", direction1Duration + "s");
        telemetry.addData("Run", runDirection1);
        
        telemetry.addLine("");
        telemetry.addLine("=== Direction 2 Settings ===");
        telemetry.addData("Power", direction2Power);
        telemetry.addData("Duration", direction2Duration + "s");
        telemetry.addData("Run", runDirection2);
    }
    
    private void stopServo() {
        if (crServo != null) {
            crServo.setPower(0);
        }
    }
}

