package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.lib.Hardware.*;
import static org.firstinspires.ftc.teamcode.lib.Vector4.*;

import static java.lang.Math.*;

import android.annotation.SuppressLint;

import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.lib.Coroutine;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.Vector4;
import org.firstinspires.ftc.teamcode.lib.vision.VisionRoutine;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.*;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "9884")
public class TeleOp extends OpMode{

    Hardware robot = new Hardware();
    Coroutine.Manager manager = new Coroutine.Manager();
    VisionRoutine vision;

    private int spindexerTarget = SpindexerPosition.A_IN;

    public static boolean enableColor = false;
    public static boolean enableApril = false;

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
        manager.start(new TurretRoutine());

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
        if (enableColor){
            vision.setCamera(VisionRoutine.CameraState.ColorCamera);
            enableColor = false;
        }
        if (enableApril){
            vision.setCamera(VisionRoutine.CameraState.AprilTagCamera);
            enableApril = false;
        }
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

    private class TurretRoutine extends Coroutine{
        @Override
        public Object loop(){
            for (AprilTagDetection detection : vision.getDetections()) {
                if (detection.metadata != null) {
                    telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                    //if (!detection.metadata.name.contains("Obelisk"))
                    {
                        telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
                                detection.robotPose.getPosition().x,
                                detection.robotPose.getPosition().y,
                                detection.robotPose.getPosition().z));
                        telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
                                detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
                                detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
                                detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
                    }
                } else {
                    telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                    telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }
            telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
            telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");

            telemetry.addData("color", String.format("%08X", vision.getColor()));
            return null;
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