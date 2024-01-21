package org.firstinspires.ftc.teamcode.processors;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class BlueFinder implements VisionProcessor {
    public Rect rectLeft = new Rect(10,100,200,200);
    public Rect rectMiddle = new Rect(220, 100, 200, 200);
    public Rect rectRight = new Rect(430, 100, 200, 200);
    Selected selection = Selected.NONE;
    Mat submat = new Mat();
    Mat hsvMat = new Mat();
    Mat thresh = new Mat();
    int pixelLeft;

    int pixelMiddle;
    int pixelRight;

    Telemetry mytelemetry;
    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }

    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {

        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

        Scalar lowHSV1 = new Scalar(75, 50, 50); // lenient lower bound HSV for yellow
        Scalar highHSV1 = new Scalar(130, 255, 255); // lenient higher bound HSV for yellow
        //Mat thresh = new Mat();


        // Get a black and white image of blue objects
        Core.inRange(hsvMat, lowHSV1, highHSV1, thresh);
        //double satRectLeft = getAvgSaturation(thresh, rectLeft);
        //double satRectMiddle = getAvgSaturation(thresh, rectMiddle);
        //double satRectRight = getAvgSaturation(thresh, rectRight);

        pixelLeft = getnWhitePixel(thresh, rectLeft);

        pixelMiddle = getnWhitePixel(thresh, rectMiddle);
        pixelRight = getnWhitePixel(thresh, rectRight);

        if ((pixelLeft > pixelMiddle) && (pixelLeft > pixelRight)) {
            return Selected.LEFT;
        } else if ((pixelMiddle > pixelLeft) && (pixelMiddle > pixelRight)) {
            return Selected.MIDDLE;
        }
        return Selected.RIGHT;
        //return thresh;


    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }
    protected int getnWhitePixel(Mat input, Rect rect) {
        submat = input.submat(rect);
        return Core.countNonZero(submat);
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.GREEN);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.RED);

        //android.graphics.Rect drawRectLeft = makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx);
        //android.graphics.Rect drawRectMiddle = makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx);
        //android.graphics.Rect drawRectRight = makeGraphicsRect(rectRight, scaleBmpPxToCanvasPx);
        canvas.drawRect(makeGraphicsRect(rectLeft, scaleBmpPxToCanvasPx), selectedPaint);
        Bitmap bmp = convertMatToBitMap(thresh);
        canvas.drawBitmap(bmp,null,makeGraphicsRect(rectMiddle, scaleBmpPxToCanvasPx), selectedPaint);
      /*
        selection = (Selected) userContext;
        switch (selection) {
            case LEFT:
                canvas.drawRect(drawRectLeft, selectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case MIDDLE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, selectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
            case RIGHT:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectLeft, nonSelectedPaint);
                canvas.drawRect(drawRectMiddle, nonSelectedPaint);
                canvas.drawRect(drawRectRight, nonSelectedPaint);
                break;
        }
        */

    }

    public Selected getSelection() {
        return selection;
    }

    public enum Selected { NONE, LEFT, MIDDLE, RIGHT }
public void print_selection(){
    mytelemetry.addData(">", "LEFT pixels %d", pixelLeft);
    mytelemetry.addData(">", "MIDDLE pixels %d", pixelMiddle);
    mytelemetry.addData(">", "RIGHT pixels %d", pixelRight);
if (selection == Selected.NONE) {
    mytelemetry.addData(">", "NONE");
} else if (selection == Selected.LEFT) {
    mytelemetry.addData(">", "LEFT");

}
else if (selection == Selected.MIDDLE) {
    mytelemetry.addData(">", "MIDDLE");

}
else if (selection == Selected.RIGHT) {
    mytelemetry.addData(">", "RIGHT");

}
}
public void setTelemetry(Telemetry telemetry){
        mytelemetry = telemetry;
}
    private static Bitmap convertMatToBitMap(Mat input){
        Bitmap bmp = null;
        Mat rgb = input;//new Mat();
       // Imgproc.cvtColor(input, rgb, Imgproc.COLOR_BGR2RGB);

            bmp = Bitmap.createBitmap(rgb.cols(), rgb.rows(), Bitmap.Config.ARGB_8888);
            Utils.matToBitmap(rgb, bmp);
        return bmp;
    }
}
