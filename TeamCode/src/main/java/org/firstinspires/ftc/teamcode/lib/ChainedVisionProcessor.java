package org.firstinspires.ftc.teamcode.lib;

import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.Rect;
import android.os.Build;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.test.CameraTest;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.LinkedList;
import java.util.List;

public class ChainedVisionProcessor implements VisionProcessor{
    private final List<VisionProcessor> _processors;

    private ChainedVisionProcessor(LinkedHashSet<VisionProcessor> processors){
        this._processors = Collections.unmodifiableList(new ArrayList<>(processors));
    }

    public static class Builder{
        private final LinkedHashSet<VisionProcessor> _processors;

        public Builder(){
            _processors = new LinkedHashSet<>();
        }

        public Builder addProcessor(VisionProcessor processor){
            _processors.add(processor);
            return this;
        }

        public Builder addProcessors(VisionProcessor... processors){
            _processors.addAll(Arrays.asList(processors));
            return this;
        }

        public Builder addProcessors(@NonNull Iterable<VisionProcessor> processors){
            for(VisionProcessor processor:
                processors){
                _processors.add(processor);
            }
            return this;
        }

        public ChainedVisionProcessor build(){
            return new ChainedVisionProcessor(_processors);
        }
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration){
        for(VisionProcessor processor :
                _processors){
            processor.init(width, height, calibration);
        }
    }

    private static class FrameData{
        public final Bitmap frame;
        public final Object[] contexts;

        public FrameData(Bitmap frame, Object[] contexts){
            this.frame = frame;
            this.contexts = contexts;
        }
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos){
        Bitmap b = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(frame, b);
        return new FrameData(b, _processors.stream().map((processor -> processor.processFrame(frame, captureTimeNanos))).toArray());
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext){
        FrameData data = (FrameData) userContext;
        Rect clipping = canvas.getClipBounds();

        Bitmap bitmap = Bitmap.createBitmap(onscreenWidth, onscreenHeight, data.frame.getConfig());
        canvas = ExposedCanvas.expose(canvas, bitmap);
        //L T R B
        //[1,-440][494,808]
        //

        float top = 0;
        float left = 0;
        float right = clipping.right - Math.abs(clipping.left);
        float bottom = clipping.bottom - Math.abs(clipping.top);

        canvas.drawBitmapMesh(data.frame, 1, 1, new float[]{
                left, top,
                right, top,
                left, bottom,
                right, bottom
        }, 0, null, 0, null);
        Object[] ctx = data.contexts;
        for(int i = 0; i < _processors.size(); i++){
            _processors.get(i).onDrawFrame(canvas, onscreenWidth, onscreenHeight, scaleBmpPxToCanvasPx, scaleCanvasDensity, ctx[i]);
        }
    }
}
