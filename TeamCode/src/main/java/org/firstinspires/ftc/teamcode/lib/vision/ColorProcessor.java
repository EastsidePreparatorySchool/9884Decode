package org.firstinspires.ftc.teamcode.lib.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.ColorSpace;
import android.graphics.Paint;
import android.graphics.Rect;

import androidx.annotation.ColorInt;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.Arrays;

public class ColorProcessor implements VisionProcessor{
    @ColorInt int color = 0;

    @ColorInt
    public int getColor(){
        return color;
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Scalar mean = Core.mean(frame);
        color = Color.rgb((int)mean.val[0], (int)mean.val[1], (int)mean.val[2]);
        return color;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        canvas.drawARGB(
                Color.alpha(color),
                Color.red(color),
                Color.green(color),
                Color.blue(color)
        );
    }
}
