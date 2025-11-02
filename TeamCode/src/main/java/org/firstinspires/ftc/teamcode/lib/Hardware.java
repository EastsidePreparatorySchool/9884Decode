package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.Vector4.of;
import static org.firstinspires.ftc.teamcode.lib.Quad.of;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

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
    private static class ServoInfo {
        public final int port;
        public final ServoController controller;

        public ServoInfo(Servo servo){
            port = servo.getPortNumber();
            controller = servo.getController();
        }

        public ServoInfo(CRServo servo){
            port = servo.getPortNumber();
            controller = servo.getController();
        }

        public double getPosition(){
            return controller.getServoPosition(port);
        }
    }

    public Telemetry telemetry;
    public Quad<DcMotor> driveMotors;
    public BNO055IMU imu;
    public Servo spindexer;
    public CRServo lift;
    //public AnalogInput liftEncoder;
    public DcMotor turretFlywheel;
    private ServoInfo liftInfo;
    private ServoInfo spindexerInfo;

    public static double SPEED_CONSTANT     = 0.80;
    public static double AUTO_CONSTANT      = 0.50;
    public static double SLOW_MODE_CONSTANT = 0.10;
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        this.telemetry.setMsTransmissionInterval(50);
        this.telemetry.addData("TransmissionMs", telemetry.getMsTransmissionInterval());
        this.telemetry.update();

        DcMotor driveMotorFL = hardwareMap.dcMotor.get("driveFL"),
                driveMotorFR = hardwareMap.dcMotor.get("driveFR"),
                driveMotorBL = hardwareMap.dcMotor.get("driveBL"),
                driveMotorBR = hardwareMap.dcMotor.get("driveBR");

        driveMotors = of(driveMotorFL, driveMotorFR, driveMotorBL, driveMotorBR);
        driveMotorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        resetMotors();

        spindexer = hardwareMap.servo.get("spindexer");
        spindexerInfo = new ServoInfo(spindexer);
        lift = hardwareMap.crservo.get("lift");
        liftInfo = new ServoInfo(lift);
        //hardwareMap.analogInput.get("liftEncoder");
        turretFlywheel = hardwareMap.dcMotor.get("turretFlywheel");
        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        return this.telemetry;
    }

    public static class SpindexerPosition{
        public static final int A_IN = 0;
        public static final int C_OUT = 60;
        public static final int B_IN = 120;
        public static final int A_OUT = 180;
        public static final int C_IN = 240;
        public static final int B_OUT = 300;
    }

    public void revTurret(){
        turretFlywheel.setPower(1);
    }
    public void endRevTurret(){
        turretFlywheel.setPower(0);
    }

    public void setSpindexer(double degrees){
        spindexer.setPosition(degrees / 360d);
    }

    public double getLiftPos(){
        return liftInfo.getPosition();
    }

    public double getSpindexerPos(){
        return spindexerInfo.getPosition();
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