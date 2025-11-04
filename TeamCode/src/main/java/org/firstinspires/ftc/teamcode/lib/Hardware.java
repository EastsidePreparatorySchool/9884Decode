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

    public static final double SPINDEXER_DEGREE_RANGE = 300.0;
    public static final double SPINDEXER_ROTATION_TIME = 0.3;
    public static final double LIFT_UP_TIME = 3.5;
    public static final double LIFT_DOWN_TIME = 2.0;
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
<<<<<<< Updated upstream
<<<<<<< Updated upstream
    public BHI260IMU imu;
    public Servo spindexer;
    public CRServo lift;
    //public AnalogInput liftEncoder;
    public DcMotor turretFlywheel;
    private ServoInfo liftInfo;

    public static double SPEED_CONSTANT     = 1.00;
    public static double AUTO_CONSTANT      = 0.50;
=======
    public BNO055IMU imu; //internal measurement unit, measures angle in which the motors are turning

=======
    public BNO055IMU imu; //internal measurement unit, measures angle in which the motors are turning

>>>>>>> Stashed changes
    public static double SPEED_CONSTANT     = 0.80; //sets speed to 80 percent of max
    public static double AUTO_CONSTANT      = 0.50; //in autonomous the robot needs to go slower, so 50%
    public static double SLOW_MODE_CONSTANT = 0.10;
    public static int SPROCKET_CONSTANT  = 15;
    public static double SPROCKET_SLOW_MULT = 1d/2;
    public static int SPROCKET_REST = -100;
    public static int SPROCKET_WALL = -422;
    public static final int ACTUATOR_SAFETY_BUFFER = 150;
    public static final int ACTUATOR_MAX_POS = -6100;
    public static final double ANGLE_ETA = Math.PI / 16; //the closest the angles of the motors need to be to be good >:)
<<<<<<< Updated upstream
>>>>>>> Stashed changes
=======
>>>>>>> Stashed changes

    //in teleop code already
    public static final Vector4 VDrive  = of(+1d, +1d, +1d, +1d);
    public static final Vector4 VStrafe = of(+1d, -1d, -1d, +1d);
    public static final Vector4 VTurn   = of(+1d, -1d, +1d, -1d);

    /**
     * This is effectively the constructor
     * @return the new {@link MultipleTelemetry} instance, recommended to reassign telemetry
     */
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry){return init(hardwareMap, telemetry, false);}
<<<<<<< Updated upstream
<<<<<<< Updated upstream
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
=======
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry, boolean auto){ //called when pressing init in teleop
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //creating an IMU class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; //sets measurement of the IMU to radians
        imu.initialize(parameters); //initializing the imu

        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(50); //how often the robot controller sends data to the driver station in milliseconds
        this.telemetry.addLine("Initialization Status Successful");
        this.telemetry.addLine("Transmission Interval:" + telemetry.getMsTransmissionInterval()); //ideally should print 50
        this.telemetry.update(); //update telemetry

=======
    public Telemetry init(HardwareMap hardwareMap, Telemetry telemetry, boolean auto){ //called when pressing init in teleop
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        imu = hardwareMap.get(BNO055IMU.class, "imu"); //creating an IMU class
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS; //sets measurement of the IMU to radians
        imu.initialize(parameters); //initializing the imu

        this.telemetry = telemetry;
        this.telemetry.setMsTransmissionInterval(50); //how often the robot controller sends data to the driver station in milliseconds
        this.telemetry.addLine("Initialization Status Successful");
        this.telemetry.addLine("Transmission Interval:" + telemetry.getMsTransmissionInterval()); //ideally should print 50
        this.telemetry.update(); //update telemetry

>>>>>>> Stashed changes
        //gets all motors
        DcMotor driveMotorFL = hardwareMap.dcMotor.get("DriveFL"),
                driveMotorFR = hardwareMap.dcMotor.get("DriveFR"),
                driveMotorBL = hardwareMap.dcMotor.get("DriveBL"),
                driveMotorBR = hardwareMap.dcMotor.get("DriveBR");
>>>>>>> Stashed changes



        driveMotors = of(driveMotorFL, driveMotorFR, driveMotorBL, driveMotorBR);
        //reverses motor direction for FR and BR so that the robot goes forward
        driveMotorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        driveMotorBR.setDirection(DcMotorSimple.Direction.REVERSE);


        for (DcMotor motor : driveMotors) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //runs based on the code only
            motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //when the motors have zero power, the robot is on BRAKE which means the wheels cannot rotate
        }
        resetMotors();

        spindexer = hardwareMap.servo.get("spindexer");
        //when we get the five turn servo, we need to set this to [0.0, 0.4] (2 turns)
        spindexer.scaleRange(0.0, 1.0);
        setSpindexer(0);

        lift = hardwareMap.crservo.get("lift");
        liftInfo = new ServoInfo(lift);

        turretFlywheel = hardwareMap.dcMotor.get("turretFly");
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
        spindexer.setPosition(degrees / SPINDEXER_DEGREE_RANGE);
    }

    public double getLiftPos(){
        return liftInfo.getPosition();
    }

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