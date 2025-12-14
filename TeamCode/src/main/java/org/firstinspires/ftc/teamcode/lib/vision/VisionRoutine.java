package org.firstinspires.ftc.teamcode.lib.vision;

import android.graphics.Bitmap;
import android.graphics.Color;
import android.util.Size;

import androidx.annotation.ColorInt;
import androidx.annotation.Nullable;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.lib.Coroutine;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

@Config
public final class VisionRoutine extends Coroutine implements CameraStreamSource{
    public static int frameWidth = 1280;
    public static int frameHeight = 720;


    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private final CameraStreamProcessor streamer;
    private final ColorProcessor color;
    private final ChainedVisionProcessor chained;

    private final WebcamName aprilCam, colorCam;

    public enum CameraState{
        AprilTagCamera,
        ColorCamera
    }


    @Nullable
    private ArrayList<AprilTagDetection> detections = null;

    private VisionRoutine(AprilTagProcessor _processor, VisionPortal _portal, ChainedVisionProcessor _chained, CameraStreamProcessor _streamer, ColorProcessor _color, WebcamName _aprilCam, WebcamName _colorCam){
        processor = _processor;
        portal = _portal;
        chained = _chained;
        streamer = _streamer;
        color = _color;
        aprilCam = _aprilCam;
        colorCam = _colorCam;

        portal.setProcessorEnabled(color, false);
    }

    @Override
    public Object loop(){
        detections = processor.getDetections();

        return getDetections();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation){
        streamer.getFrameBitmap(continuation);
    }

    public List<AprilTagDetection> getDetections(){
        return detections == null ? List.of() : Collections.unmodifiableList(detections);
    }

    @ColorInt
    public int getColor(){
        return color.getColor();
    }

    public CameraState getCamera(){
        return portal.getActiveCamera() == aprilCam ? CameraState.AprilTagCamera : CameraState.ColorCamera;
    }

    public void toggleCamera(){
        CameraState state = getCamera();
        CameraState newState = state == CameraState.ColorCamera ? CameraState.AprilTagCamera : CameraState.ColorCamera;
        setCamera(newState);
    }

    public void setCamera(CameraState camera){
        portal.setProcessorEnabled(chained, camera == CameraState.AprilTagCamera);
        portal.setProcessorEnabled(color, camera == CameraState.ColorCamera);
        portal.setActiveCamera(camera == CameraState.AprilTagCamera ? aprilCam : colorCam);
    }

    public static class Builder{
        private boolean autoStreamToDashAfterBuild = false;
        private double fps;
        private Position position = new Position();
        private YawPitchRollAngles rotation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        private final AprilTagProcessor.Builder processor;
        private final VisionPortal.Builder portal;

        private WebcamName aprilCam, colorCam;

        public Builder(){
            processor = new AprilTagProcessor.Builder()
                    .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES);
            portal = new VisionPortal.Builder();
        }

        /// Units are centimeters
        public Builder setPosition(double x, double y, double z){
            position = new Position(DistanceUnit.CM, x, y, z, 0);
            processor.setCameraPose(position, rotation);
            return this;
        }

        /// Units are degrees
        /// {@link AprilTagProcessor.Builder#setCameraPose(Position, YawPitchRollAngles)}
        public Builder setRotation(double yaw, double pitch, double roll){
            rotation = new YawPitchRollAngles(AngleUnit.DEGREES, yaw, pitch, roll, 0);
            processor.setCameraPose(position, rotation);
            return this;
        }

        public Builder autoStreamToDashOnBuild(double fps){
            autoStreamToDashAfterBuild = true;
            this.fps = fps;
            return this;
        }

        public Builder setCameras(WebcamName aprilCam, WebcamName colorCam){
            this.aprilCam = aprilCam;
            this.colorCam = colorCam;
            portal.setCamera(ClassFactory.getInstance()
                    .getCameraManager().nameForSwitchableCamera(aprilCam, colorCam));
            return this;
        }

        public VisionRoutine build(){
            processor.setDrawAxes(true)
                    .setDrawTagOutline(true)
                    .setDrawTagID(true);

            AprilTagProcessor builtProc = processor.build();

            CameraStreamProcessor streamer = new CameraStreamProcessor();
            ChainedVisionProcessor chained = new ChainedVisionProcessor.Builder()
                    .addProcessors(builtProc, streamer)
                    .build();
            ColorProcessor color = new ColorProcessor();

            VisionRoutine built = new VisionRoutine(
                    builtProc,
                    portal.addProcessor(chained)
                            .addProcessor(color)
                            .setAutoStartStreamOnBuild(true)
                            .setCameraResolution(new Size(frameWidth, frameHeight))
                            .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                        .build(),
                    chained,
                    streamer,
                    color,
                    aprilCam,
                    colorCam);

            if (autoStreamToDashAfterBuild)
                FtcDashboard.getInstance().startCameraStream(built, fps);
            return built;
        }
    }

    public void close(){
        portal.close();
    }
}
