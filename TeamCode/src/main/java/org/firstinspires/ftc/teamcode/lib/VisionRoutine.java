package org.firstinspires.ftc.teamcode.lib;

import android.graphics.Bitmap;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.function.Consumer;
import org.firstinspires.ftc.robotcore.external.function.Continuation;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Objects;

public final class VisionRoutine extends Coroutine implements CameraStreamSource{
    private final AprilTagProcessor processor;
    private final VisionPortal portal;
    private final CameraStreamProcessor streamer;

    @Nullable
    private ArrayList<AprilTagDetection> detections = null;

    private VisionRoutine(AprilTagProcessor _processor, VisionPortal _portal, CameraStreamProcessor _streamer){
        processor = _processor;
        portal = _portal;
        streamer = _streamer;
    }

    @Override
    public Object loop(){
        detections = processor.getFreshDetections();

        return getDetections();
    }

    @Override
    public void getFrameBitmap(Continuation<? extends Consumer<Bitmap>> continuation){
        streamer.getFrameBitmap(continuation);
    }

    public List<AprilTagDetection> getDetections(){
        return detections == null ? List.of() : Collections.unmodifiableList(detections);
    }

    public static class Builder{
        private boolean streamToDash = false;
        private Position position = new Position();
        private YawPitchRollAngles rotation = new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 0, 0);

        private final AprilTagProcessor.Builder processor;
        private final VisionPortal.Builder portal;

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

        /// Fixes streaming to FtcDash using black magic
        public Builder enableStreaming(){
            streamToDash = true;
            processor.setDrawAxes(true)
                    .setDrawTagOutline(true)
                    .setDrawTagID(true);
            portal.setAutoStartStreamOnBuild(true);
            return this;
        }

        public Builder setCamera(BuiltinCameraDirection cam){
            portal.setCamera(cam);
            return this;
        }

        public Builder setCamera(CameraName cam){
            portal.setCamera(cam);
            return this;
        }

        public VisionRoutine Build(){
            AprilTagProcessor builtProc = processor.build();
            VisionPortal builtPortal;
            CameraStreamProcessor streamer = null;
            if (streamToDash){
                streamer = new CameraStreamProcessor();
                builtPortal = portal.addProcessor(
                        new ChainedVisionProcessor.Builder()
                            .addProcessors(builtProc, streamer)
                            .build()
                        ).build();

            } else{
                builtPortal = portal.addProcessor(builtProc).build();
            }
            return new VisionRoutine(builtProc, builtPortal, streamer);
        }
    }

    public void close(){
        portal.close();
    }
}
