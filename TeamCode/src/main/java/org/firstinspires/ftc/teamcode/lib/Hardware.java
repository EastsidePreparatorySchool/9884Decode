package org.firstinspires.ftc.teamcode.lib;

import static org.firstinspires.ftc.teamcode.lib.Vector4.of;
import static org.firstinspires.ftc.teamcode.lib.Quad.of;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.bosch.BHI260IMU;
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

    public static final double SPINDEXER_DEGREE_RANGE = 330;
    public static final double SPINDEXER_ROTATION_TIME = 0.3;
    public static double FLICKER_FLICK_TIME = 0.3;
    public static final double TURRET_REV_UP_TIME = 1.0;
    public static final double TURRET_REV_DOWN_TIME = 1.0;

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
    public BHI260IMU imu;
    public Servo spindexer;
    public DcMotor turretFlywheel;
    public DcMotor intakeFlywheel;
    public DcMotor turretBase;
    public Servo flicker;
    public Servo hood;


    public static double SPEED_CONSTANT     = 1.00;
    public static double AUTO_CONSTANT      = 0.50;

    public static final Vector4 VDrive  = of(+1d, +1d, +1d, +1d);
    public static final Vector4 VStrafe = of(+1d, -1d, -1d, +1d);
    public static final Vector4 VTurn   = of(+1d, -1d, +1d, -1d);

    /**
     * This is effectively the constructor
     * @return the new {@link MultipleTelemetry} instance, recommended to reassign telemetry
     */
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry){return init(hardwareMap, telemetry, false);}
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry, boolean auto){
        imu = hardwareMap.get(BHI260IMU.class, "imu");
        imu.initialize();

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
        //when we get the five turn servo, we need to set this to [0.0, 0.4] (2 turns)
        spindexer.scaleRange(0.0, 1.0);
        setSpindexer(0);

        flicker = hardwareMap.servo.get("flicker");
        flicker.scaleRange(0.8, 1);
        unflick();
        hood = hardwareMap.servo.get("hood");
        hood.scaleRange(0.1, 0.45);

        turretFlywheel = hardwareMap.dcMotor.get("turretFly");
        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeFlywheel = hardwareMap.dcMotor.get("intakeFly");
        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turretBase = hardwareMap.dcMotor.get("turretBase");

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
        turretFlywheel.setPower(-1);
    }
    public void endRevTurret(){
        turretFlywheel.setPower(0);
    }

    public void setSpindexer(double degrees){
        spindexer.setPosition(degrees / SPINDEXER_DEGREE_RANGE);
    }

    public void flick(){
        flicker.setPosition(0);
    }

    public void unflick(){
        flicker.setPosition(1);
    }

//    public double getLiftPos(){
//        return liftInfo.getPosition();
//    }

    /**
     * Applies powers to each drive motor based on the components of power
     * @see DcMotor#setPower(double)
     * @see Vector4
     */
    public void powerMotors(Vector4 power){
        //Remember that motors cannot have a power above 1
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
     * Resents motor encodes and stops motors. If goal is solely to stop motors, use {@link Hardware#stopMotors()} instead
     * @see DcMotor#setMode(DcMotor.RunMode)
     * @see DcMotor.RunMode#STOP_AND_RESET_ENCODER
     */
    public void resetMotors(){
        for(DcMotor motor : driveMotors){
            motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    /**
     * @return the distance between two angles, mod 2pi
     */
    public static double angularDist(double a, double b){
        return (a - b) % (2 * Math.PI);
    }

    /**
     * @return the current heading of the imu, from -PI to PI
     * Returns 0 if IMU is not available
     */
    public float getHeading(){
        return imu.getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
    }

    /**
     * Logs the positions of all drive motors to telemetry
     * @see Telemetry#addData(String, Object)
     */
    public void logMotorPos(){
        telemetry.addData("posFR", driveMotors.w.getCurrentPosition());
        telemetry.addData("posBR", driveMotors.x.getCurrentPosition());
        telemetry.addData("posFL", driveMotors.y.getCurrentPosition());
        telemetry.addData("posBL", driveMotors.z.getCurrentPosition());
    }

    public void logHeading(){
        telemetry.addData("theta", getHeading());
    }
}