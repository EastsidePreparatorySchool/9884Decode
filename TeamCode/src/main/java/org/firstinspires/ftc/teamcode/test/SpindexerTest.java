package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Simple test for Spindexer servo (servo5, named "spindexer")
 * 
 * Moves servo all the way left or right
 */
@Autonomous(name = "Spindexer Test", group = "Test")
@Config
public class SpindexerTest extends LinearOpMode {
    
    // Servo Configuration
    public static String servoName = "spindexer";  // Servo name as configured
    
    // Controls
    public static boolean moveLeft = false;   // Set to true to move all the way left
    public static boolean moveRight = false;  // Set to true to move all the way right
    
    // State tracking
    private static boolean lastMoveLeft = false;
    private static boolean lastMoveRight = false;
    
    private Servo spindexer;
    
    @Override
    public void runOpMode() {
        telemetry = new com.acmerobotics.dashboard.telemetry.MultipleTelemetry(
            telemetry, 
            FtcDashboard.getInstance().getTelemetry()
        );
        telemetry.setMsTransmissionInterval(50);
        
        telemetry.addLine("Spindexer Test");
        telemetry.addLine("Configure in Dashboard at: 192.168.43.1:8080/dash");
        telemetry.addLine("");
        telemetry.addLine("Set moveLeft=true to move all the way left (0.0)");
        telemetry.addLine("Set moveRight=true to move all the way right (1.0)");
        telemetry.update();
        
        waitForStart();
        
        // Initialize servo - try multiple name variations
        spindexer = null;
        String[] nameVariations = {servoName, "servo5", "Servo5", "Servo 5", "servo 5"};
        
        for (String name : nameVariations) {
            try {
                spindexer = hardwareMap.get(Servo.class, name);
                if (spindexer != null) {
                    telemetry.addLine("Spindexer initialized: '" + name + "'");
                    // Start at middle position
                    spindexer.setPosition(0.5);
                    telemetry.addLine("Set to middle position");
                    break;
                }
            } catch (Exception e) {
                // Try next variation
            }
        }
        
        if (spindexer == null) {
            telemetry.addLine("ERROR: Spindexer not found!");
            telemetry.addLine("Tried names: " + String.join(", ", nameVariations));
            telemetry.addLine("Configure '" + servoName + "' as Servo in Robot Configuration");
        }
        telemetry.update();
        sleep(1000);
        
        while (opModeIsActive()) {
            // Check for move left
            if (moveLeft && !lastMoveLeft) {
                if (spindexer != null) {
                    spindexer.setPosition(0.0);
                    lastMoveLeft = true;
                } else {
                    telemetry.addLine("ERROR: Spindexer not initialized!");
                }
            } else if (!moveLeft) {
                lastMoveLeft = false;
            }
            
            // Check for move right
            if (moveRight && !lastMoveRight) {
                if (spindexer != null) {
                    spindexer.setPosition(1.0);
                    lastMoveRight = true;
                } else {
                    telemetry.addLine("ERROR: Spindexer not initialized!");
                }
            } else if (!moveRight) {
                lastMoveRight = false;
            }
            
            // Display status
            displayStatus();
            
            telemetry.update();
            sleep(50);
        }
    }
    
    private void displayStatus() {
        telemetry.clearAll();
        telemetry.addLine("=== Spindexer Test ===");
        telemetry.addLine("Servo: " + servoName);
        telemetry.addLine("");
        telemetry.addLine("Controls:");
        telemetry.addData("Move Left (0.0)", moveLeft);
        telemetry.addData("Move Right (1.0)", moveRight);
    }
}

