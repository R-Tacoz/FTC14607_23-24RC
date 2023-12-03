package org.firstinspires.ftc.teamcode.CV;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.util.UtilityCameraFrameCapture;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

public class SpikeDetectProcessorBlue implements VisionProcessor {

    public enum SpikePosition {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }
    private volatile SpikePosition detectedPosition = SpikePosition.NONE; //position with the most amount of the color
    private double detectedPercentage = 0.00;
    private Mat currentInput = new Mat();

    private final Scalar
            lower_red_bounds = new Scalar(155, 55, 55),
            upper_red_bounds = new Scalar(179, 225, 255);
    private final Scalar
            lower_blue_bounds = new Scalar(100, 0, 0),
            upper_blue_bounds = new Scalar(255,50,50);
    private final Scalar
            lower_purple_bounds = new Scalar(100,0,100),
            upper_purple_bounds = new Scalar(255, 100, 255);

    private double leftPercent, centerPercent, rightPercent;

    private Mat blurredMat = new Mat();
    private Mat leftMat = new Mat(), centerMat = new Mat(), rightMat = new Mat();
    private Mat leftMatPurple = new Mat(), centerMatPurple = new Mat(), rightMatPurple = new Mat();
    private Rect leftBounds = new Rect(
            new Point(0,0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH / 3, UtilityCameraFrameCapture.RESOLUTION_HEIGHT )
            ),
    centerBounds = new Rect(
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH / 3, 0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH * 2 / 3 , UtilityCameraFrameCapture.RESOLUTION_HEIGHT)
            ),
    rightBounds = new Rect(
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH * 2 / 3, 0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH,UtilityCameraFrameCapture.RESOLUTION_HEIGHT));

    private Rect fullBounds = new Rect(
            new Point(0,0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT));

    @Override
    public void init(int width, int height, CameraCalibration calibration){

    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV);
        Imgproc.blur(input, blurredMat, new Size(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT));
        blurredMat = blurredMat.submat(fullBounds);

        leftMat = input.submat(leftBounds);
        centerMat = input.submat(centerBounds);
        rightMat = input.submat(rightBounds);

        Core.inRange(leftMat, lower_blue_bounds, upper_blue_bounds, leftMatPurple);
        Core.inRange(centerMat, lower_blue_bounds, upper_blue_bounds, centerMatPurple);
        Core.inRange(rightMat, lower_blue_bounds, upper_blue_bounds, rightMatPurple);

        double total_pixels = 640*480;
        leftPercent = Core.countNonZero(leftMatPurple) / total_pixels;
        centerPercent = Core.countNonZero(centerMatPurple) / total_pixels;
        rightPercent = Core.countNonZero(rightMatPurple) / total_pixels;

        double maxPercent = leftPercent == 0 && centerPercent == 0 && rightPercent == 0 ? -1 : Math.max(leftPercent, Math.max(centerPercent, rightPercent));

        detectedPercentage = maxPercent;

        if(maxPercent == leftPercent){
            detectedPosition = SpikePosition.LEFT;
            Imgproc.rectangle(
                    input,
                    new Point(0,0),
                    new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH / 3, UtilityCameraFrameCapture.RESOLUTION_HEIGHT),
                    lower_blue_bounds);
        }
        else if(maxPercent == centerPercent){
            detectedPosition = SpikePosition.CENTER;
        }
        else if(maxPercent == rightPercent){
            detectedPosition = SpikePosition.RIGHT;
        }
        else{
            detectedPosition = SpikePosition.NONE;
        }


        currentInput = input;

        //stuff
        return blurredMat;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        // Not useful either
    }

    public SpikePosition getSpikePosition() {
        return detectedPosition;
    }

    public double getSpikePercent(){
        return detectedPercentage;
    }

    public Mat getCurrentInput(){
        return currentInput;
    }

    public Mat getLeftMat(){ return leftMat; }
    public Mat getCenterMat(){ return centerMat; }
    public Mat getRightMat(){ return rightMat; }
    public Mat getBlurredMat(){ return blurredMat;}

    public Mat getLeftMatPurple() { return leftMatPurple; }
    public Mat getCenterMatPurple() { return centerMatPurple; }
    public Mat getRightMatPurple() { return rightMatPurple; }
}
