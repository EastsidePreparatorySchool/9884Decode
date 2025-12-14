package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.health.connect.datatypes.SexualActivityRecord;

import androidx.annotation.FloatRange;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Coroutine;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;
import org.firstinspires.ftc.teamcode.lib.vision.VisionRoutine;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;

import java.util.*;
import java.util.stream.Collectors;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends OpMode{

    Hardware robot = new Hardware();
    Coroutine.Manager manager = new Coroutine.Manager();
    VisionRoutine vision;

    private int spindexerTarget = SpindexerPosition.A_IN;

    public static int aprilTarget = 24;

    public static int obelisk = 0;

    public static double hoodSpeed = 0.05;

    public static int turretMax = -50;
    public static int turretMin = -800;
    public static int turretSafetyRange = 50;
    public static double turretSpeed = 0.2;
    public static double turretSafeSpeed = 0.15;
    public static double turretEpsilon = 0.05;

    // ========== USER-TUNABLE PID ==========
    // Error is normalized to [-1..1] where + means target is left of center (turn left).
    public static double kP = 0.75;
    public static double kI = 0.10;
    public static double kD = 0.12;

    // Integrate only when close-ish to target (prevents windup while far away / no target)
    public static double iZone = 0.6;          // normalized error
    public static double iMax  = 0.35;         // integral clamp (already multiplied by kI later)
    public static double dAlpha = 0.6;         // derivative low-pass (1.0 = no filtering)


    @Override
    public void init(){
        telemetry = robot.init(hardwareMap, telemetry);
        telemetry.update();

        vision = new VisionRoutine.Builder()
                .setCameras(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setPosition(0, 0, 0)
                .setRotation(0, 0, 0)
                .autoStreamToDashOnBuild(30)
                .build();

        manager.start(new DriveTrainRoutine());
        manager.start(new SpindexerRoutine());
        manager.start(vision);
        manager.start(new ColorRoutine());
        manager.start(new TurretRoutine());
        manager.start(new IntakeRoutine());
        manager.start(new HoodRoutine());

        robot.turretBase.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turretBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        manager.disableAllExcept(vision);
    }

    @Override
    public void init_loop(){
        manager.loop(getRuntime());
        Optional<AprilTagDetection> opt = vision.getDetections().stream().filter(d -> d.id >= 21 && d.id <= 23).findAny();
        opt.ifPresent(aprilTagDetection -> obelisk = aprilTagDetection.id);
    }

    @Override
    public void start(){
        manager.enableAll();
        robot.turretBase.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.turretBase.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        manager.loop(getRuntime());
        telemetry.update();
    }

    @Override
    public void stop(){
        FtcDashboard.getInstance().stopCameraStream();
        robot.turretFlywheel.setPower(0);
        robot.intakeFlywheel.setPower(0);
        vision.close();
    }

    class ColorRoutine extends Coroutine{

        @Nullable
        @Override
        protected Object loop(){
            if (gamepad2.bWasPressed()){
                //vision.toggleCamera();
            }

            float[] hsv = new float[3];
            Color.colorToHSV(vision.getColor(), hsv);

            telemetry.addData("h", hsv[0]);
            telemetry.addData("s", hsv[1]);
            telemetry.addData("v", hsv[2]);

            return null;
        }
    }

    class HoodRoutine extends Coroutine{
        @FloatRange(from = 0, to = 1)
        double pos = 0;

        @Nullable
        @Override
        protected Object loop(){
            if(gamepad2.dpad_down){
                pos -= hoodSpeed;
            }
            if (gamepad2.dpad_up){
                pos += hoodSpeed;
            }
            pos = Math.max(0, Math.min(1, pos));
            robot.hood.setPosition(pos);

            return null;
        }
    }

    class IntakeRoutine extends Coroutine{

        @Nullable
        @Override
        protected Object loop(){
            robot.intakeFlywheel.setPower(gamepad2.right_trigger * (gamepad2.a ? -1 : 1));

            return null;
        }
    }

    class TurretRoutine extends Coroutine{
        private double integral = 0.0;
        private double prevErr = 0.0;
        private double prevT = Double.NaN;
        private double dFilt = 0.0;

        private void resetPid(){
            integral = 0.0;
            prevErr = 0.0;
            prevT = Double.NaN;
            dFilt = 0.0;
        }

        @SuppressLint("DefaultLocale")
        @Override
        public Object loop(){
            Optional<AprilTagDetection> optional = vision.getDetections().stream().filter(d -> d.id == aprilTarget).findAny();
            if (!optional.isPresent()){
                // Manual turret control when target is not visible (gamepad2 bumpers).
                double out = 0.0;
                if (gamepad2.left_stick_x != 0){
                    out = -gamepad2.left_stick_x * turretSpeed;
                } else {
                    robot.turretBase.setPower(0);
                    resetPid();
                    telemetry.addData("turretMode", "idle (no target)");
                    return null;
                }

                final int pos = robot.turretBase.getCurrentPosition();

                // safety: never drive past hard limits
                if (out < 0 && (pos - turretMin) <= 0) out = 0;
                if (out > 0 && (turretMax - pos) <= 0) out = 0;

                // safety: reduce speed near limits
                if (out < 0 && (pos - turretMin) <= turretSafetyRange){
                    if (out < -turretSafeSpeed) out = -turretSafeSpeed;
                }
                if (out > 0 && (turretMax - pos) <= turretSafetyRange){
                    if (out > turretSafeSpeed) out = turretSafeSpeed;
                }

                robot.turretBase.setPower(out);
                resetPid();

                telemetry.addData("turretMode", "manual");
                telemetry.addData("turretPos", pos);
                telemetry.addData("turretOut", out);
                return null;
            }

            telemetry.addData("turretMode", "auto");

            AprilTagDetection detection = optional.get();

            final double centerX = VisionRoutine.frameWidth / 2d;
            final double errorPx = centerX - detection.center.x;          // + => target is left of center
            final double error   = errorPx / centerX;                      // normalize to [-1..1]

            telemetry.addData("dx", errorPx);
            telemetry.addData("dxN", error);
            telemetry.addData("d", detection.ftcPose.z);

            final int pos = robot.turretBase.getCurrentPosition();
            telemetry.addData("turretPos", pos);

            // deadband in pixels (keeps turret from buzzing)
            if (Math.abs(errorPx) < turretEpsilon * VisionRoutine.frameWidth){
                robot.turretBase.setPower(0);
                resetPid();
                return null;
            }

            // dt
            final double now = getRuntime();
            final double dt = (Double.isNaN(prevT) ? 0.0 : (now - prevT));
            prevT = now;

            // integral with iZone + clamp
            if (dt > 0.0 && Math.abs(error) <= iZone){
                integral += error * dt;
                if (integral > iMax) integral = iMax;
                if (integral < -iMax) integral = -iMax;
            } else {
                integral = 0.0;
            }

            // derivative (filtered)
            double d = 0.0;
            if (dt > 0.0) d = (error - prevErr) / dt;
            prevErr = error;
            dFilt = dAlpha * d + (1.0 - dAlpha) * dFilt;

            // PID output
            double out = getOut(error, pos);

            robot.turretBase.setPower(out);

            telemetry.addData("tPID_p", kP * error);
            telemetry.addData("tPID_i", kI * integral);
            telemetry.addData("tPID_d", kD * dFilt);
            telemetry.addData("tPID_out", out);

            return null;
        }

        private double getOut(double error, int pos){
            double out = (kP * error) + (kI * integral) + (kD * dFilt);

            // clamp to your configured turret speed limits
            if (out > turretSpeed) out = turretSpeed;
            if (out < -turretSpeed) out = -turretSpeed;

            // safety: never drive past hard limits
            if (out < 0 && (pos - turretMin) <= 0) out = 0;
            if (out > 0 && (turretMax - pos) <= 0) out = 0;

            // safety: reduce speed near limits (same intent as previous bang-bang code)
            if (out < 0 && (pos - turretMin) <= turretSafetyRange){
                if (out < -turretSafeSpeed) out = -turretSafeSpeed;
            }
            if (out > 0 && (turretMax - pos) <= turretSafetyRange){
                if (out > turretSafeSpeed) out = turretSafeSpeed;
            }
            return out;
        }
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
                    state = 12;
                    return Coroutine.Yield.delay(Math.max(TURRET_REV_UP_TIME, SPINDEXER_ROTATION_TIME));
                case 12:
                    robot.flick();
                    state = 4;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 4:
                    robot.unflick();
                    state = 5;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 5:
                    robot.setSpindexer(SpindexerPosition.A_OUT);
                    state = 6;
                    return Coroutine.Yield.delay(SPINDEXER_ROTATION_TIME);
                case 6:
                    robot.flick();
                    state = 7;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 7:
                    robot.unflick();
                    state = 8;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 8:
                    robot.setSpindexer(SpindexerPosition.C_OUT);
                    state = 9;
                    return Coroutine.Yield.delay(SPINDEXER_ROTATION_TIME);
                case 9:
                    robot.flick();
                    state = 10;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 10:
                    robot.unflick();
                    state = 11;
                    return Coroutine.Yield.delay(FLICKER_FLICK_TIME);
                case 11:
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

//            telemetry.addData("lx", lx);
//            telemetry.addData("ly", ly);
//            telemetry.addData("rx", rx);

//            telemetry.addLine(VDrive.toString());
//            telemetry.addLine(mult(VDrive, 1).toString());

            ArrayList<Vector4> powerList = new ArrayList<>();
            powerList.add(mult(VDrive, ly));
            powerList.add(mult(VStrafe, lx));
            powerList.add(mult(VTurn, rx));

            Vector4 power = powerList.stream().reduce((a, b) -> add(a, b)).get().mult(SPEED_CONSTANT);
            //scale such that no motor is powered outside of [-1.0,1.0]
            power.div(max(1, power.stream().max().orElse(0) / SPEED_CONSTANT));
            power.mult(1 - 0.5 * gamepad1.right_trigger);
            robot.powerMotors(power);

//            telemetry.addData("power w", power.w);
//            telemetry.addData("power x", power.x);
//            telemetry.addData("power y", power.y);
//            telemetry.addData("power z", power.z);

//            robot.logMotorPos();
            robot.logHeading();

            return null;
        }
    }
}