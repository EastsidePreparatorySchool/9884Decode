package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;
import android.graphics.Color;
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.internal.camera.names.WebcamNameImpl;
import org.firstinspires.ftc.teamcode.lib.vision.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.lib.vision.ChainedVisionProcessor;
import org.firstinspires.ftc.teamcode.lib.vision.ColorProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 */
@Config
@TeleOp(name = "Vision Test", group = "9884")
public class CameraTest extends LinearOpMode {

    private AprilTagProcessor processor;
    private VisionPortal portal;
    private CameraStreamProcessor streamer;
    private ColorProcessor colorProcessor;
    private ChainedVisionProcessor chainedProcessor;

    private WebcamName aprilCam;
    private WebcamName colorCam;

    boolean isInColorMode = false;

    public static double camx = 0;
    public static double camy = 0;
    public static double camz = 0;
    public static double camyaw = 0;
    public static double campitch = 0;
    public static double camroll = 0;

    @Override
    public void runOpMode() {
        FtcDashboard instance = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, instance.getTelemetry());

        aprilCam = hardwareMap.get(WebcamName.class, "Webcam 1");
        colorCam = hardwareMap.get(WebcamName.class, "Webcam 2");

        buildVision();
        instance.startCameraStream(streamer, 30);
        waitForStart();
        while (opModeIsActive()) {
            telemetryAprilTag();
            sleep(100);
            telemetry.addData("color", String.format("%08X", colorProcessor.getColor()));

            if (gamepad1.aWasPressed()){
                isInColorMode = !isInColorMode;
                portal.setProcessorEnabled(chainedProcessor, !isInColorMode);
                portal.setProcessorEnabled(colorProcessor, isInColorMode);
                portal.setActiveCamera(isInColorMode ? colorCam : aprilCam);
            }

            telemetry.update();
        }
        portal.close();
    }

    private void buildVision() {

        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setCameraPose(new Position(DistanceUnit.MM, camx, camy, camz, 0), new YawPitchRollAngles(AngleUnit.DEGREES, camyaw, campitch, camroll, 0))
                .build();

        streamer = new CameraStreamProcessor();

        colorProcessor = new ColorProcessor();

        chainedProcessor = new ChainedVisionProcessor.Builder()
                .addProcessor(processor)
                .addProcessor(streamer)
                .build();

        portal = new VisionPortal.Builder()
                .setCamera(ClassFactory.getInstance()
                        .getCameraManager().nameForSwitchableCamera(aprilCam, colorCam))
                .addProcessor(chainedProcessor)
                .addProcessor(colorProcessor)
                .setAutoStartStreamOnBuild(true)
                .build();
        portal.setProcessorEnabled(colorProcessor, false);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = processor.getDetections();

        for (AprilTagDetection detection : currentDetections) {
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

    }

}
