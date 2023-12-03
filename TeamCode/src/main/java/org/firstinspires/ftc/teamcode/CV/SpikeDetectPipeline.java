package org.firstinspires.ftc.teamcode.CV;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.UtilityCameraFrameCapture;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class SpikeDetectPipeline extends OpenCvPipeline {

    public enum SpikePosition {
        NONE,
        LEFT,
        CENTER,
        RIGHT
    }
    private volatile SpikePosition detectedPosition = SpikePosition.NONE; //position with the most amount of the color
    private double detectedPercentage = 0.00;
    private Mat currentInput = new Mat();

    private final Scalar PURPLE = new Scalar(138,43, 226);
    private final Scalar
            lower_purple_bounds = new Scalar(186,85,221) ,
            upper_purple_bounds = new Scalar(75,0,130);

    private double leftPercent, centerPercent, rightPercent;

    private Mat blurredMat = new Mat();
    private Mat leftMat = new Mat(), centerMat = new Mat(), rightMat = new Mat();
    private Mat leftMatPurple = new Mat(), centerMatPurple = new Mat(), rightMatPurple = new Mat();
    private Rect leftBounds = new Rect(
            new Point(0,0),
            new Point((int)(UtilityCameraFrameCapture.RESOLUTION_WIDTH / 3),UtilityCameraFrameCapture.RESOLUTION_HEIGHT)),
    centerBounds = new Rect(
            new Point(0,0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH * 2 / 3,UtilityCameraFrameCapture.RESOLUTION_HEIGHT)),
    rightBounds = new Rect(
            new Point(0,0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH,UtilityCameraFrameCapture.RESOLUTION_HEIGHT));

    private Rect fullBounds = new Rect(
            new Point(0,0),
            new Point(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT));

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, input, Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(input, blurredMat, new Size(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT));
        blurredMat = blurredMat.submat( fullBounds );

        leftMat = blurredMat.submat(leftBounds);
        centerMat = blurredMat.submat(centerBounds);
        rightMat = blurredMat.submat(rightBounds);

        Core.inRange(leftMat, lower_purple_bounds, upper_purple_bounds, leftMatPurple);
        Core.inRange(centerMat, lower_purple_bounds, upper_purple_bounds, centerMatPurple);
        Core.inRange(rightMat, lower_purple_bounds, upper_purple_bounds, rightMatPurple);

        leftPercent = Core.countNonZero(leftMatPurple);
        centerPercent = Core.countNonZero(centerMatPurple);
        rightPercent = Core.countNonZero(rightMatPurple);

        double maxPecent = Math.max(leftPercent, Math.max(centerPercent, rightPercent) );

        detectedPercentage = maxPecent;

        if(maxPecent == leftPercent){
            detectedPosition = SpikePosition.LEFT;
        }
        else if(maxPecent == centerPercent){
            detectedPosition = SpikePosition.CENTER;
        }
        else if(maxPecent == rightPercent){
            detectedPosition = SpikePosition.RIGHT;
        }

        blurredMat.release();
        leftMat.release();
        centerMat.release();
        rightMat.release();
        leftMatPurple.release();
        centerMatPurple.release();
        rightMatPurple.release();

        currentInput = input;

        //stuff
        return input;
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
}
