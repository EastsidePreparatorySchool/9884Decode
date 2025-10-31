package org.firstinspires.ftc.teamcode.test;

import android.annotation.SuppressLint;
import android.os.Build;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.lib.CameraStreamProcessor;
import org.firstinspires.ftc.teamcode.lib.ChainedVisionProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_intro/apriltag-intro.html
 * https://ftc-docs.firstinspires.org/apriltag-detection-values
 */
@Config
@TeleOp(name = "April Tag", group = "9884")
public class CameraTest extends LinearOpMode {

    private AprilTagProcessor processor;
    private VisionPortal portal;
    private CameraStreamProcessor streamer;

    private static final Position camPos = new Position(DistanceUnit.MM, 0, 0, 0, 0);
    private static final YawPitchRollAngles camRot = new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0);

    public static Telemetry t;

    @Override
    public void runOpMode() {
        FtcDashboard instance = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, instance.getTelemetry());
        t = telemetry;

        telemetry.addData("ver", Build.VERSION.SDK);

        buildVision();
        instance.startCameraStream(streamer, 30);
        waitForStart();
        while (opModeIsActive()) {
            telemetryAprilTag();
            sleep(100);
            telemetry.update();
        }
        portal.close();
    }

    private void buildVision() {
        processor = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setCameraPose(camPos, camRot)
                .build();

        streamer = new CameraStreamProcessor();

        portal = new VisionPortal.Builder()
                .setCamera(BuiltinCameraDirection.BACK)

                .addProcessor(new ChainedVisionProcessor.Builder()
                        .addProcessor(processor)
                        .addProcessor(streamer)
                        .build())
//                .addProcessor(processor)
//                .addProcessor(streamer)
                .setAutoStartStreamOnBuild(true)
                .build();
    }

    private void buildVisionEasy(){
        processor = AprilTagProcessor.easyCreateWithDefaults();
        portal = VisionPortal.easyCreateWithDefaults(BuiltinCameraDirection.BACK, processor);
    }

    @SuppressLint("DefaultLocale")
    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = processor.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));

                if (!detection.metadata.name.contains("Obelisk"))
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
