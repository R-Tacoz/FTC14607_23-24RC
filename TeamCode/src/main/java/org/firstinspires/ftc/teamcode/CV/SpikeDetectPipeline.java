package org.firstinspires.ftc.teamcode.CV;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
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
    private volatile SpikePosition detectedPosition = SpikePosition.NONE;

    public static int REGION_WIDTH = 40;
    public static int REGION_HEIGHT = 80;

    private final Scalar PURPLE = new Scalar();

    private double leftPercent, centerPercent, rightPercent;
    private Mat leftMat = new Mat(), centerMat = new Mat(), rightMat = new Mat();

    @Override
    public Mat processFrame(Mat input) {
        //stuff
        return input;
    }

    public SpikePosition getSpikePosition() {
        return detectedPosition;
    }
}
