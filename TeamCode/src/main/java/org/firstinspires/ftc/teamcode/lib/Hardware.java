package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.Vector4.of;
import static org.firstinspires.ftc.teamcode.lib.Quad.of;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

/**
 * Everything you need is in this class (except some utilities)
 * You should have a good reason for using things other than this
 */
@Config // ftc dash: 192.168.43.1:8080/dash
public final class Hardware{
    public Telemetry telemetry;
    public Quad<DcMotor> driveMotors;
    public BNO055IMU imu;

    public static double SPEED_CONSTANT     = 0.80;
    public static double AUTO_CONSTANT      = 0.50;
    public static double SLOW_MODE_CONSTANT = 0.10;
    public static int SPROCKET_CONSTANT  = 15;
    public static double SPROCKET_SLOW_MULT = 1d/2;
    public static int SPROCKET_REST = -100;
    public static int SPROCKET_WALL = -422;
    public static final int ACTUATOR_SAFETY_BUFFER = 150;
    public static final int ACTUATOR_MAX_POS = -6100;
    public static final double ANGLE_ETA = Math.PI / 16;

    public static final Vector4 VDrive  = of(+1d, +1d, +1d, +1d);
    public static final Vector4 VStrafe = of(+1d, -1d, -1d, +1d);
    public static final Vector4 VTurn   = of(+1d, -1d, +1d, -1d);

    /**
     * This is effectively the constructor
     * @return the new {@link MultipleTelemetry} instance, recommended to reassign telemetry
     */
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry){return init(hardwareMap, telemetry, false);}
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry, boolean auto){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        try {
            imu = hardwareMap.get(BNO055IMU.class, "imu");
            BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
            imu.initialize(parameters);
            telemetry.addLine("IMU initialized successfully");
        } catch (Exception e) {
            imu = null;
            telemetry.addLine("WARNING: IMU not found - robot will work but heading features disabled");
            telemetry.addLine("To enable: Configure 'imu' as BNO055IMU in Robot Configuration");
        }

        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(50);
        this.telemetry.addLine("Initialization Status Successful");
        this.telemetry.addLine("Transmission Interval:" + telemetry.getMsTransmissionInterval());
        
        // Debug: List all available DC motors
        telemetry.addLine("--- Available DC Motors ---");
        for (String name : hardwareMap.getAllNames(DcMotor.class)) {
            telemetry.addLine("Found motor: '" + name + "'");
        }
        telemetry.update();

        try {
            // Try multiple name variations - hardwareMap.get() might throw exception or return null
            DcMotor driveMotorFL = null, driveMotorFR = null, driveMotorBL = null, driveMotorBR = null;
            
            // Helper function to safely try getting a motor
            java.util.function.Function<String, DcMotor> tryGetMotor = (name) -> {
                try {
                    return hardwareMap.dcMotor.get(name);
                } catch (Exception e) {
                    return null;
                }
            };
            
            // Try all variations for FL (Front Left)
            String[] flNames = {"Drive FL", "DriveFL", "driveFL", "drive FL", "drive_fl", "Drive_FL", "FrontLeft", "frontLeft", "front_left"};
            for (String name : flNames) {
                driveMotorFL = tryGetMotor.apply(name);
                if (driveMotorFL != null) break;
            }
            
            // Try all variations for FR (Front Right)
            String[] frNames = {"Drive FR", "DriveFR", "driveFR", "drive FR", "drive_fr", "Drive_FR", "FrontRight", "frontRight", "front_right"};
            for (String name : frNames) {
                driveMotorFR = tryGetMotor.apply(name);
                if (driveMotorFR != null) break;
            }
            
            // Try all variations for BL (Back Left)
            String[] blNames = {"Drive BL", "DriveBL", "driveBL", "drive BL", "drive_bl", "Drive_BL", "BackLeft", "backLeft", "back_left"};
            for (String name : blNames) {
                driveMotorBL = tryGetMotor.apply(name);
                if (driveMotorBL != null) break;
            }
            
            // Try all variations for BR (Back Right)
            String[] brNames = {"Drive BR", "DriveBR", "driveBR", "drive BR", "drive_br", "Drive_BR", "BackRight", "backRight", "back_right"};
            for (String name : brNames) {
                driveMotorBR = tryGetMotor.apply(name);
                if (driveMotorBR != null) break;
            }
            
            // If any motor is still null, try case-insensitive search through all available motors
            if (driveMotorFL == null || driveMotorFR == null || driveMotorBL == null || driveMotorBR == null) {
                java.util.SortedSet<String> allMotorNames = hardwareMap.getAllNames(DcMotor.class);
                telemetry.addLine("Searching through all available motors (case-insensitive)...");
                
                for (String motorName : allMotorNames) {
                    String lowerName = motorName.toLowerCase().replaceAll("[^a-z]", "");
                    if (driveMotorFL == null && (lowerName.contains("frontleft") || lowerName.contains("drivefl") || lowerName.contains("fl") && !lowerName.contains("fr"))) {
                        try { driveMotorFL = hardwareMap.dcMotor.get(motorName); } catch (Exception ignored) {}
                    }
                    if (driveMotorFR == null && (lowerName.contains("frontright") || lowerName.contains("drivefr") || lowerName.contains("fr"))) {
                        try { driveMotorFR = hardwareMap.dcMotor.get(motorName); } catch (Exception ignored) {}
                    }
                    if (driveMotorBL == null && (lowerName.contains("backleft") || lowerName.contains("drivebl") || lowerName.contains("bl") && !lowerName.contains("br"))) {
                        try { driveMotorBL = hardwareMap.dcMotor.get(motorName); } catch (Exception ignored) {}
                    }
                    if (driveMotorBR == null && (lowerName.contains("backright") || lowerName.contains("drivebr") || lowerName.contains("br"))) {
                        try { driveMotorBR = hardwareMap.dcMotor.get(motorName); } catch (Exception ignored) {}
                    }
                }
            }
            
            // If any motor is still null, throw exception with helpful message
            if (driveMotorFL == null || driveMotorFR == null || driveMotorBL == null || driveMotorBR == null) {
                String missing = "";
                if (driveMotorFL == null) missing += "Front Left (FL), ";
                if (driveMotorFR == null) missing += "Front Right (FR), ";
                if (driveMotorBL == null) missing += "Back Left (BL), ";
                if (driveMotorBR == null) missing += "Back Right (BR), ";
                missing = missing.substring(0, missing.length() - 2);
                
                telemetry.addLine("ERROR: Missing motors: " + missing);
                telemetry.addLine("Please configure motors on the driver hub with names like:");
                telemetry.addLine("  'Drive FL', 'Drive FR', 'Drive BL', 'Drive BR'");
                telemetry.addLine("  OR 'DriveFL', 'DriveFR', 'DriveBL', 'DriveBR'");
                telemetry.update();
                throw new IllegalStateException("Missing drive motors: " + missing + ". Check telemetry for available motor names and configure your robot.");
            }

            driveMotors = of(driveMotorFL, driveMotorFR, driveMotorBL, driveMotorBR);
            driveMotorFR.setDirection(DcMotorSimple.Direction.REVERSE);
            driveMotorBR.setDirection(DcMotorSimple.Direction.REVERSE);
            for (DcMotor motor : driveMotors) {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            }
            resetMotors();
            telemetry.addLine("Drive motors initialized successfully");
        } catch (Exception e) {
            telemetry.addLine("ERROR: Drive motors not found!");
            telemetry.addLine("Required motors: Drive FL, Drive FR, Drive BL, Drive BR");
            telemetry.addLine("Exception: " + e.getMessage());
            telemetry.update();
            throw new IllegalStateException("Drive motors must be configured in Robot Configuration", e);
        }
        return telemetry;
    }

    /**
     * Applies powers to each drive motor based on the components of power
     * @see DcMotor#setPower(double)
     * @see Vector4
     */
    public void powerMotors(Vector4 power){
        //Remember that motors cannot have a power above 1
        if (driveMotors == null) {
            telemetry.addLine("ERROR: driveMotors not initialized!");
            return;
        }
        driveMotors.w.setPower(power.w);
        driveMotors.x.setPower(power.x);
        driveMotors.y.setPower(power.y);
        driveMotors.z.setPower(power.z);
    }

    /**
     * Applies equal power to each drive motor
     * @see DcMotor#setPower(double)
     */
    public void powerMotors(double power){
        powerMotors(repeat(power));
    }

    /**
     * Stops all drive motors
     * @see DcMotor#setPower(double)
     */
    public void stopMotors() {powerMotors(0);}

    //Util

    /**
     * Causes execution to sleep for a given time
     * @see Thread#sleep(long)
     * @see System#currentTimeMillis()
     */
    public void sleep(long ms){
        long start = System.currentTimeMillis();
        //noinspection StatementWithEmptyBody
        while(System.currentTimeMillis() < start + ms){}
    }

    /**
     * Resents motor encodes and stops motors. If goal is solely to stop motors, use {@link Hardware#stopMotors()} instead
     * @see DcMotor#setMode(DcMotor.RunMode)
     * @see DcMotor.RunMode#STOP_AND_RESET_ENCODER
     */
    public void resetMotors(){
        if (driveMotors == null) return;
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * Drives robot forward for specific time
     */
    public void moveTime(Vector4 dir, double scalar, int ms){
        powerMotors(mult(dir, AUTO_CONSTANT * scalar));
        sleep(ms);
        stopMotors();
    }

    /**
     * Turns the robot by an angle, positive is right
     * @param phase angle to turn by IN DEGREES
     */
    @Deprecated
    public void turn(double phase){
        //angles are mod 2pi
        phase *= (-1 * Math.toRadians(phase)) % (Math.PI * 2);
        powerMotors(mult(VTurn, -Math.signum(phase) * AUTO_CONSTANT));
        double target = (getHeading() + phase) % (Math.PI * 2);
        while(angularDist(phase, target) > ANGLE_ETA){
            telemetry.addLine("awaiting turn to " + target + ". Current : " + getHeading());
            telemetry.addData("dist", angularDist(phase, target));
            telemetry.update();
        }
        stopMotors();
    }

    /**
     * @return the distance between two angles, mod 2pi
     */
    public static double angularDist(double a, double b){
        return (a - b) % (2 * Math.PI);
    }

    /** @deprecated */
    public int getEncoderDistance(){
        if (driveMotors == null) return 0;
        return driveMotors.w.getCurrentPosition();
    }

    /** @deprecated */
    public void moveEncoderDistance(int targetDistance){
        //Pro-tip: listen to no quarters by AB-XY/Chirpy Chips when you get stuck
        resetMotors();
        while(getEncoderDistance() < targetDistance){
            logMotorPos();
            powerMotors(AUTO_CONSTANT);
        }
        resetMotors();
    }

    /**
     * @return the current heading of the imu, from -PI to PI
     * Returns 0 if IMU is not available
     * @see BNO055IMU#getAngularOrientation(AxesReference, AxesOrder, AngleUnit)
     */
    public float getHeading(){
        if (imu == null) return 0;
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    public void rotateTime(int t, float d){
        powerMotors(mult(VTurn, d * SLOW_MODE_CONSTANT));
        sleep(t);
    }
    public void rotate(float to){
        powerMotors(mult(VTurn, SLOW_MODE_CONSTANT * SLOW_MODE_CONSTANT));
        while(angularDist(getHeading(), to) > ANGLE_ETA){
            telemetry.addData("angledist", angularDist(getHeading(), to));
            logHeading();
        }
        stopMotors();
    }

    /**
     * Logs the positions of all drive motors to telemetry
     * @see Telemetry#addData(String, Object)
     */
    public void logMotorPos(){
        if (driveMotors == null) {
            telemetry.addData("posFR", "N/A - motors not initialized");
            return;
        }
        telemetry.addData("posFR", driveMotors.w.getCurrentPosition());
        telemetry.addData("posBR", driveMotors.x.getCurrentPosition());
        telemetry.addData("posFL", driveMotors.y.getCurrentPosition());
        telemetry.addData("posBL", driveMotors.z.getCurrentPosition());
    }

    public void logHeading(){
        if (imu != null) {
            telemetry.addData("theta", getHeading());
        } else {
            telemetry.addData("theta", "IMU not available");
        }
    }
}