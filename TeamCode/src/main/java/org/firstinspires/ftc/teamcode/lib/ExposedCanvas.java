package org.firstinspires.ftc.teamcode.lib;

import android.graphics.*;
import android.graphics.fonts.Font;
import android.graphics.text.MeasuredText;
import android.os.Build;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.annotation.RequiresApi;

/**
 Allows any draw calls to be applied to both this canvas, and a secondary canvas.
 Provides a view into this canvas' bitmap which **should** be equal to the secondary canvas'.
 * @see android.graphics.Canvas
 * @noinspection deprecation
 */
public class ExposedCanvas extends Canvas {
    private final Canvas base;
    private final Bitmap view;

    public static ExposedCanvas expose(Canvas canvas) {
        Bitmap bitmap = Bitmap.createBitmap(canvas.getWidth(), canvas.getHeight(), Bitmap.Config.ARGB_8888);
        return new ExposedCanvas(canvas, bitmap);
    }

    public static ExposedCanvas expose(Canvas canvas, Bitmap bitmap){
        return new ExposedCanvas(canvas, bitmap);
    }

    private ExposedCanvas(Canvas canvas, Bitmap bitmap) {
        super(bitmap);
        this.base = canvas;
        this.view = bitmap;
    }

    public Bitmap copy() {
        return view.copy(Bitmap.Config.ARGB_8888, false);
    }

    // ---- Clipping ----

//    @Override
//    public boolean clipOutPath(@NonNull Path path) {
//        super.clipOutPath(path);
//        return base.clipOutPath(path);
//    }
//
//    @Override
//    public boolean clipOutRect(@NonNull Rect rect) {
//        super.clipOutRect(rect);
//        return base.clipOutRect(rect);
//    }
//
//    @Override
//    public boolean clipOutRect(@NonNull RectF rect) {
//        super.clipOutRect(rect);
//        return base.clipOutRect(rect);
//    }
//
//    @Override
//    public boolean clipOutRect(float left, float top, float right, float bottom) {
//        super.clipOutRect(left, top, right, bottom);
//        return base.clipOutRect(left, top, right, bottom);
//    }
//
//    @Override
//    public boolean clipOutRect(int left, int top, int right, int bottom) {
//        super.clipOutRect(left, top, right, bottom);
//        return base.clipOutRect(left, top, right, bottom);
//    }
//
//    @Override
//    public void clipOutShader(@NonNull Shader shader) {
//        super.clipOutShader(shader);
//        base.clipOutShader(shader);
//    }

    @Override
    public boolean clipPath(@NonNull Path path) {
        super.clipPath(path);
        return base.clipPath(path);
    }

    @Override
    public boolean clipPath(@NonNull Path path, @NonNull Region.Op op) {
        super.clipPath(path, op);
        return base.clipPath(path, op);
    }

    @Override
    public boolean clipRect(@NonNull Rect rect) {
        super.clipRect(rect);
        return base.clipRect(rect);
    }

    @Override
    public boolean clipRect(@NonNull Rect rect, @NonNull Region.Op op) {
        super.clipRect(rect, op);
        return base.clipRect(rect, op);
    }

    @Override
    public boolean clipRect(@NonNull RectF rect) {
        super.clipRect(rect);
        return base.clipRect(rect);
    }

    @Override
    public boolean clipRect(@NonNull RectF rect, @NonNull Region.Op op) {
        super.clipRect(rect, op);
        return base.clipRect(rect, op);
    }

    @Override
    public boolean clipRect(float left, float top, float right, float bottom) {
        super.clipRect(left, top, right, bottom);
        return base.clipRect(left, top, right, bottom);
    }

    @Override
    public boolean clipRect(float left, float top, float right, float bottom, @NonNull Region.Op op) {
        super.clipRect(left, top, right, bottom, op);
        return base.clipRect(left, top, right, bottom, op);
    }

    @Override
    public boolean clipRect(int left, int top, int right, int bottom) {
        super.clipRect(left, top, right, bottom);
        return base.clipRect(left, top, right, bottom);
    }

//    @Override
//    public void clipShader(@NonNull Shader shader) {
//        super.clipShader(shader);
//        base.clipShader(shader);
//    }

    // ---- Matrix / State ----

    @Override
    public void concat(@Nullable Matrix matrix) {
        super.concat(matrix);
        base.concat(matrix);
    }

//    @Override
//    public void concat(@Nullable Matrix44 m) {
//        super.concat(m);
//        base.concat(m);
//    }
//
//    @Override
//    public void disableZ() {
//        super.disableZ();
//        base.disableZ();
//    }
//
//    @Override
//    public void enableZ() {
//        super.enableZ();
//        base.enableZ();
//    }

    @Override
    public void restore() {
        super.restore();
        base.restore();
    }

    @Override
    public void restoreToCount(int saveCount) {
        super.restoreToCount(saveCount);
        base.restoreToCount(saveCount);
    }

    @Override
    public void rotate(float degrees) {
        super.rotate(degrees);
        base.rotate(degrees);
    }

    @Override
    public int save() {
        int a = super.save();
        int b = base.save();
        return Math.min(a, b);
    }

    @Override
    public int saveLayer(@Nullable RectF bounds, @Nullable Paint paint) {
        int a = super.saveLayer(bounds, paint);
        base.saveLayer(bounds, paint);
        return a;
    }

    @Override
    public int saveLayer(@Nullable RectF bounds, @Nullable Paint paint, int saveFlags) {
        int a = super.saveLayer(bounds, paint, saveFlags);
        base.saveLayer(bounds, paint, saveFlags);
        return a;
    }

    @Override
    public int saveLayer(float left, float top, float right, float bottom, @Nullable Paint paint) {
        int a = super.saveLayer(left, top, right, bottom, paint);
        base.saveLayer(left, top, right, bottom, paint);
        return a;
    }

    @Override
    public int saveLayer(float left, float top, float right, float bottom, @Nullable Paint paint, int saveFlags) {
        int a = super.saveLayer(left, top, right, bottom, paint, saveFlags);
        base.saveLayer(left, top, right, bottom, paint, saveFlags);
        return a;
    }

    @Override
    public int saveLayerAlpha(@Nullable RectF bounds, int alpha) {
        int a = super.saveLayerAlpha(bounds, alpha);
        base.saveLayerAlpha(bounds, alpha);
        return a;
    }

    @Override
    public int saveLayerAlpha(@Nullable RectF bounds, int alpha, int saveFlags) {
        int a = super.saveLayerAlpha(bounds, alpha, saveFlags);
        base.saveLayerAlpha(bounds, alpha, saveFlags);
        return a;
    }

    @Override
    public int saveLayerAlpha(float left, float top, float right, float bottom, int alpha) {
        int a = super.saveLayerAlpha(left, top, right, bottom, alpha);
        base.saveLayerAlpha(left, top, right, bottom, alpha);
        return a;
    }

    @Override
    public int saveLayerAlpha(float left, float top, float right, float bottom, int alpha, int saveFlags) {
        int a = super.saveLayerAlpha(left, top, right, bottom, alpha, saveFlags);
        base.saveLayerAlpha(left, top, right, bottom, alpha, saveFlags);
        return a;
    }

    @Override
    public void scale(float sx, float sy) {
        super.scale(sx, sy);
        base.scale(sx, sy);
    }

    @Override
    public void setBitmap(@Nullable Bitmap bitmap) {
        super.setBitmap(bitmap);
        base.setBitmap(bitmap);
    }

    @Override
    public void setDensity(int density) {
        super.setDensity(density);
        base.setDensity(density);
    }

    @Override
    public void setDrawFilter(@Nullable DrawFilter filter) {
        super.setDrawFilter(filter);
        base.setDrawFilter(filter);
    }

    @Override
    public void setMatrix(@Nullable Matrix matrix) {
        super.setMatrix(matrix);
        base.setMatrix(matrix);
    }

    @Override
    public void skew(float sx, float sy) {
        super.skew(sx, sy);
        base.skew(sx, sy);
    }

    @Override
    public void translate(float dx, float dy) {
        super.translate(dx, dy);
        base.translate(dx, dy);
    }

    // ---- Drawing ----

    @Override
    public void drawARGB(int a, int r, int g, int b) {
        super.drawARGB(a, r, g, b);
        base.drawARGB(a, r, g, b);
    }

    @Override
    public void drawArc(@NonNull RectF oval, float startAngle, float sweepAngle, boolean useCenter, @NonNull Paint paint) {
        super.drawArc(oval, startAngle, sweepAngle, useCenter, paint);
        base.drawArc(oval, startAngle, sweepAngle, useCenter, paint);
    }

    @Override
    public void drawArc(float left, float top, float right, float bottom, float startAngle, float sweepAngle, boolean useCenter, @NonNull Paint paint) {
        super.drawArc(left, top, right, bottom, startAngle, sweepAngle, useCenter, paint);
        base.drawArc(left, top, right, bottom, startAngle, sweepAngle, useCenter, paint);
    }

    @Override
    public void drawBitmap(@NonNull Bitmap bitmap, @NonNull Matrix matrix, @Nullable Paint paint) {
        super.drawBitmap(bitmap, matrix, paint);
        base.drawBitmap(bitmap, matrix, paint);
    }

    @Override
    public void drawBitmap(@NonNull Bitmap bitmap, @Nullable Rect src, @NonNull Rect dst, @Nullable Paint paint) {
        super.drawBitmap(bitmap, src, dst, paint);
        base.drawBitmap(bitmap, src, dst, paint);
    }

    @Override
    public void drawBitmap(@NonNull Bitmap bitmap, @Nullable Rect src, @NonNull RectF dst, @Nullable Paint paint) {
        super.drawBitmap(bitmap, src, dst, paint);
        base.drawBitmap(bitmap, src, dst, paint);
    }

    @Override
    public void drawBitmap(@NonNull Bitmap bitmap, float left, float top, @Nullable Paint paint) {
        super.drawBitmap(bitmap, left, top, paint);
        base.drawBitmap(bitmap, left, top, paint);
    }

    @Override
    public void drawBitmap(@NonNull int[] colors, int offset, int stride, float x, float y, int width, int height, boolean hasAlpha, @Nullable Paint paint) {
        super.drawBitmap(colors, offset, stride, x, y, width, height, hasAlpha, paint);
        base.drawBitmap(colors, offset, stride, x, y, width, height, hasAlpha, paint);
    }

    @Override
    public void drawBitmap(@NonNull int[] colors, int offset, int stride, int x, int y, int width, int height, boolean hasAlpha, @Nullable Paint paint) {
        super.drawBitmap(colors, offset, stride, x, y, width, height, hasAlpha, paint);
        base.drawBitmap(colors, offset, stride, x, y, width, height, hasAlpha, paint);
    }

    @Override
    public void drawBitmapMesh(@NonNull Bitmap bitmap, int meshWidth, int meshHeight, @NonNull float[] verts, int vertOffset, @Nullable int[] colors, int colorOffset, @Nullable Paint paint) {
        super.drawBitmapMesh(bitmap, meshWidth, meshHeight, verts, vertOffset, colors, colorOffset, paint);
        base.drawBitmapMesh(bitmap, meshWidth, meshHeight, verts, vertOffset, colors, colorOffset, paint);
    }

    @Override
    public void drawCircle(float cx, float cy, float radius, @NonNull Paint paint) {
        super.drawCircle(cx, cy, radius, paint);
        base.drawCircle(cx, cy, radius, paint);
    }

    @Override
    public void drawColor(int color) {
        super.drawColor(color);
        base.drawColor(color);
    }

//    @Override
//    public void drawColor(int color, @NonNull BlendMode mode) {
//        super.drawColor(color, mode);
//        base.drawColor(color, mode);
//    }

    @Override
    public void drawColor(int color, @NonNull PorterDuff.Mode mode) {
        super.drawColor(color, mode);
        base.drawColor(color, mode);
    }

//    @Override
//    public void drawColor(long color) {
//        super.drawColor(color);
//        base.drawColor(color);
//    }
//
//    @Override
//    public void drawColor(long color, @NonNull BlendMode mode) {
//        super.drawColor(color, mode);
//        base.drawColor(color, mode);
//    }
//
//    @Override
//    public void drawDoubleRoundRect(@NonNull RectF outer, float outerRx, float outerRy, @NonNull RectF inner, float innerRx, float innerRy, @NonNull Paint paint) {
//        super.drawDoubleRoundRect(outer, outerRx, outerRy, inner, innerRx, innerRy, paint);
//        base.drawDoubleRoundRect(outer, outerRx, outerRy, inner, innerRx, innerRy, paint);
//    }
//
//    @Override
//    public void drawDoubleRoundRect(@NonNull RectF outer, @NonNull float[] outerRadii, @NonNull RectF inner, @NonNull float[] innerRadii, @NonNull Paint paint) {
//        super.drawDoubleRoundRect(outer, outerRadii, inner, innerRadii, paint);
//        base.drawDoubleRoundRect(outer, outerRadii, inner, innerRadii, paint);
//    }
//
//    @Override
//    public void drawGlyphs(@NonNull int[] glyphIds, int glyphIdOffset, @NonNull float[] positions, int positionOffset, int glyphCount, @NonNull Font font, @NonNull Paint paint) {
//        super.drawGlyphs(glyphIds, glyphIdOffset, positions, positionOffset, glyphCount, font, paint);
//        base.drawGlyphs(glyphIds, glyphIdOffset, positions, positionOffset, glyphCount, font, paint);
//    }

    @Override
    public void drawLine(float startX, float startY, float stopX, float stopY, @NonNull Paint paint) {
        super.drawLine(startX, startY, stopX, stopY, paint);
        base.drawLine(startX, startY, stopX, stopY, paint);
    }

    @Override
    public void drawLines(@NonNull float[] pts, @NonNull Paint paint) {
        super.drawLines(pts, paint);
        base.drawLines(pts, paint);
    }

    @Override
    public void drawLines(@NonNull float[] pts, int offset, int count, @NonNull Paint paint) {
        super.drawLines(pts, offset, count, paint);
        base.drawLines(pts, offset, count, paint);
    }

//    @Override
//    public void drawMesh(@NonNull Mesh mesh, @Nullable BlendMode blendMode, @NonNull Paint paint) {
//        super.drawMesh(mesh, blendMode, paint);
//        base.drawMesh(mesh, blendMode, paint);
//    }

    @Override
    public void drawOval(@NonNull RectF oval, @NonNull Paint paint) {
        super.drawOval(oval, paint);
        base.drawOval(oval, paint);
    }

    @Override
    public void drawOval(float left, float top, float right, float bottom, @NonNull Paint paint) {
        super.drawOval(left, top, right, bottom, paint);
        base.drawOval(left, top, right, bottom, paint);
    }

    @Override
    public void drawPaint(@NonNull Paint paint) {
        super.drawPaint(paint);
        base.drawPaint(paint);
    }

//    @Override
//    public void drawPatch(@NonNull NinePatch patch, @NonNull Rect dst, @Nullable Paint paint) {
//        super.drawPatch(patch, dst, paint);
//        base.drawPatch(patch, dst, paint);
//    }
//
//    @Override
//    public void drawPatch(@NonNull NinePatch patch, @NonNull RectF dst, @Nullable Paint paint) {
//        super.drawPatch(patch, dst, paint);
//        base.drawPatch(patch, dst, paint);
//    }

    @Override
    public void drawPath(@NonNull Path path, @NonNull Paint paint) {
        super.drawPath(path, paint);
        base.drawPath(path, paint);
    }

    @Override
    public void drawPicture(@NonNull Picture picture) {
        super.drawPicture(picture);
        base.drawPicture(picture);
    }

    @Override
    public void drawPicture(@NonNull Picture picture, @NonNull Rect dst) {
        super.drawPicture(picture, dst);
        base.drawPicture(picture, dst);
    }

    @Override
    public void drawPicture(@NonNull Picture picture, @NonNull RectF dst) {
        super.drawPicture(picture, dst);
        base.drawPicture(picture, dst);
    }

    @Override
    public void drawPoint(float x, float y, @NonNull Paint paint) {
        super.drawPoint(x, y, paint);
        base.drawPoint(x, y, paint);
    }

    @Override
    public void drawPoints(@NonNull float[] pts, @NonNull Paint paint) {
        super.drawPoints(pts, paint);
        base.drawPoints(pts, paint);
    }

    @Override
    public void drawPoints(float[] pts, int offset, int count, @NonNull Paint paint) {
        super.drawPoints(pts, offset, count, paint);
        base.drawPoints(pts, offset, count, paint);
    }

    @Override
    public void drawPosText(@NonNull char[] text, int index, int count, @NonNull float[] pos, @NonNull Paint paint) {
        super.drawPosText(text, index, count, pos, paint);
        base.drawPosText(text, index, count, pos, paint);
    }

    @Override
    public void drawPosText(@NonNull String text, @NonNull float[] pos, @NonNull Paint paint) {
        super.drawPosText(text, pos, paint);
        base.drawPosText(text, pos, paint);
    }

    @Override
    public void drawRGB(int r, int g, int b) {
        super.drawRGB(r, g, b);
        base.drawRGB(r, g, b);
    }

    @Override
    public void drawRect(@NonNull Rect r, @NonNull Paint paint) {
        super.drawRect(r, paint);
        base.drawRect(r, paint);
    }

    @Override
    public void drawRect(@NonNull RectF rect, @NonNull Paint paint) {
        super.drawRect(rect, paint);
        base.drawRect(rect, paint);
    }

    @Override
    public void drawRect(float left, float top, float right, float bottom, @NonNull Paint paint) {
        super.drawRect(left, top, right, bottom, paint);
        base.drawRect(left, top, right, bottom, paint);
    }

//    @Override
//    public void drawRenderNode(@NonNull RenderNode renderNode) {
//        super.drawRenderNode(renderNode);
//        base.drawRenderNode(renderNode);
//    }

    @Override
    public void drawRoundRect(@NonNull RectF rect, float rx, float ry, @NonNull Paint paint) {
        super.drawRoundRect(rect, rx, ry, paint);
        base.drawRoundRect(rect, rx, ry, paint);
    }

    @Override
    public void drawRoundRect(float left, float top, float right, float bottom, float rx, float ry, @NonNull Paint paint) {
        super.drawRoundRect(left, top, right, bottom, rx, ry, paint);
        base.drawRoundRect(left, top, right, bottom, rx, ry, paint);
    }

    @Override
    public void drawText(@NonNull char[] text, int index, int count, float x, float y, @NonNull Paint paint) {
        super.drawText(text, index, count, x, y, paint);
        base.drawText(text, index, count, x, y, paint);
    }

    @Override
    public void drawText(@NonNull CharSequence text, int start, int end, float x, float y, @NonNull Paint paint) {
        super.drawText(text, start, end, x, y, paint);
        base.drawText(text, start, end, x, y, paint);
    }

    @Override
    public void drawText(@NonNull String text, float x, float y, @NonNull Paint paint) {
        super.drawText(text, x, y, paint);
        base.drawText(text, x, y, paint);
    }

    @Override
    public void drawText(@NonNull String text, int start, int end, float x, float y, @NonNull Paint paint) {
        super.drawText(text, start, end, x, y, paint);
        base.drawText(text, start, end, x, y, paint);
    }

    @Override
    public void drawTextOnPath(@NonNull char[] text, int index, int count, @NonNull Path path, float hOffset, float vOffset, @NonNull Paint paint) {
        super.drawTextOnPath(text, index, count, path, hOffset, vOffset, paint);
        base.drawTextOnPath(text, index, count, path, hOffset, vOffset, paint);
    }

    @Override
    public void drawTextOnPath(@NonNull String text, @NonNull Path path, float hOffset, float vOffset, @NonNull Paint paint) {
        super.drawTextOnPath(text, path, hOffset, vOffset, paint);
        base.drawTextOnPath(text, path, hOffset, vOffset, paint);
    }

//    @Override
//    public void drawTextRun(@NonNull MeasuredText text, int start, int end, int contextStart, int contextEnd, float x, float y, boolean isRtl, @NonNull Paint paint) {
//        super.drawTextRun(text, start, end, contextStart, contextEnd, x, y, isRtl, paint);
//        base.drawTextRun(text, start, end, contextStart, contextEnd, x, y, isRtl, paint);
//    }

    @Override
    public void drawTextRun(@NonNull char[] text, int index, int count, int contextIndex, int contextCount, float x, float y, boolean isRtl, @NonNull Paint paint) {
        super.drawTextRun(text, index, count, contextIndex, contextCount, x, y, isRtl, paint);
        base.drawTextRun(text, index, count, contextIndex, contextCount, x, y, isRtl, paint);
    }

    @Override
    public void drawTextRun(@NonNull CharSequence text, int start, int end, int contextStart, int contextEnd, float x, float y, boolean isRtl, @NonNull Paint paint) {
        super.drawTextRun(text, start, end, contextStart, contextEnd, x, y, isRtl, paint);
        base.drawTextRun(text, start, end, contextStart, contextEnd, x, y, isRtl, paint);
    }

    @Override
    public void drawVertices(@NonNull VertexMode mode, int vertexCount, @NonNull float[] verts, int vertOffset, @Nullable float[] texs, int texOffset, @Nullable int[] colors, int colorOffset, @Nullable short[] indices, int indexOffset, int indexCount, @NonNull Paint paint) {
        super.drawVertices(mode, vertexCount, verts, vertOffset, texs, texOffset, colors, colorOffset, indices, indexOffset, indexCount, paint);
        base.drawVertices(mode, vertexCount, verts, vertOffset, texs, texOffset, colors, colorOffset, indices, indexOffset, indexCount, paint);
    }

    // ---- Query / Info passthroughs stay as super (snapshot bitmap) ----

    @Override
    public boolean getClipBounds(@NonNull Rect bounds) {
        return super.getClipBounds(bounds);
    }

    @Override
    public int getDensity() {
        return super.getDensity();
    }

    @Nullable
    @Override
    public DrawFilter getDrawFilter() {
        return super.getDrawFilter();
    }

    @Override
    public int getHeight() {
        return super.getHeight();
    }

    @Override
    public void getMatrix(@NonNull Matrix ctm) {
        super.getMatrix(ctm);
    }

    @Override
    public int getMaximumBitmapHeight() {
        return super.getMaximumBitmapHeight();
    }

    @Override
    public int getMaximumBitmapWidth() {
        return super.getMaximumBitmapWidth();
    }

    @Override
    public int getSaveCount() {
        return super.getSaveCount();
    }

    @Override
    public int getWidth() {
        return super.getWidth();
    }

    @Override
    public boolean isHardwareAccelerated() {
        return base.isHardwareAccelerated();
    }

    @Override
    public boolean isOpaque() {
        return super.isOpaque();
    }

    // ---- QuickReject ----

//    @Override
//    public boolean quickReject(@NonNull Path path) {
//        boolean a = super.quickReject(path);
//        boolean b = base.quickReject(path);
//        return a && b;
//    }

    @Override
    public boolean quickReject(@NonNull Path path, @NonNull EdgeType type) {
        boolean a = super.quickReject(path, type);
        boolean b = base.quickReject(path, type);
        return a && b;
    }

//    @Override
//    public boolean quickReject(@NonNull RectF rect) {
//        boolean a = super.quickReject(rect);
//        boolean b = base.quickReject(rect);
//        return a && b;
//    }

    @Override
    public boolean quickReject(@NonNull RectF rect, @NonNull EdgeType type) {
        boolean a = super.quickReject(rect, type);
        boolean b = base.quickReject(rect, type);
        return a && b;
    }

//    @Override
//    public boolean quickReject(float left, float top, float right, float bottom) {
//        boolean a = super.quickReject(left, top, right, bottom);
//        boolean b = base.quickReject(left, top, right, bottom);
//        return a && b;
//    }

    @Override
    public boolean quickReject(float left, float top, float right, float bottom, @NonNull EdgeType type) {
        boolean a = super.quickReject(left, top, right, bottom, type);
        boolean b = base.quickReject(left, top, right, bottom, type);
        return a && b;
    }
}
