package org.firstinspires.ftc.teamcode;

import androidx.annotation.Nullable;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.lib.Coroutine;
import org.firstinspires.ftc.teamcode.lib.Hardware;
import org.firstinspires.ftc.teamcode.lib.vision.VisionRoutine;

public class VisionAuto extends OpMode{

    Hardware robot = new Hardware();
    Coroutine.Manager routines = new Coroutine.Manager();

    VisionRoutine vision;

    @Override
    public void init(){
        telemetry = robot.init(hardwareMap, telemetry);

        vision = new VisionRoutine.Builder()
                .setCameras(hardwareMap.get(WebcamName.class, "Webcam 1"), hardwareMap.get(WebcamName.class, "Webcam 2"))
                .setPosition(0, 0, 0)
                .setRotation(0, 0, 0)
                .autoStreamToDashOnBuild(30)
                .build();
    }

    @Override
    public void loop(){
        routines.loop(getRuntime());
    }

    private class FieldScanRoutine extends Coroutine{

        @Nullable
        @Override
        protected Object loop(){
            return null;
        }
    }

    @Override
    public void stop(){
        vision.close();
    }
}
