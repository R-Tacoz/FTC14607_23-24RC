package org.firstinspires.ftc.teamcode.util.vision;

import android.graphics.Canvas;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*
 * VisionPortal-based pipeline to detect the position and color of the team prop on the randomized
 * Spike Mark
 */
public class PropDetectProcessor implements VisionProcessor {

    public enum PropPosition {
        NONE,
        LEFT,
        CENTER,
        RIGHT,
    }

    private volatile PropPosition detectedPosition = PropPosition.NONE; //position with the most amount of the color
    private double detectionConfidence = 0.00;

    // Color boundaries for masking. Adjust as needed.
    private static final Scalar
            RED_LOWER_BOUNDS = new Scalar(155, 55, 55),
            RED_UPPER_BOUNDS = new Scalar(179, 225, 255),
            BLUE_LOWER_BOUNDS = new Scalar(100, 0, 0),
            BLUE_UPPER_BOUNDS = new Scalar(255,50,50);

    // Regions to color mask; values are proportions of the screen. Adjust as needed.
    private static final double
            LEFT_TLCORNER_X     = 0,    LEFT_TLCORNER_Y     = 0,
            LEFT_WIDTH          = 0.33, LEFT_HEIGHT         = 1,

            CENTER_TLCORNER_X   = 0.33, CENTER_TLCORNER_Y   = 0,
            CENTER_WIDTH        = 0.33, CENTER_HEIGHT       = 1,

            RIGHT_TLCORNER_X    = 0.66, RIGHT_TLCORNER_Y    = 0,
            RIGHT_WIDTH         = 0.33, RIGHT_HEIGHT        = 1;

    private Rect leftBounds, centerBounds, rightBounds, fullBounds;
    private double[] regionSizes; // # pixels in each region

    // Indices 0,1,2 correspond to left, center, right
    private Rect[] regionBounds;
    private double[] confidences;
    private double leftPercent, centerPercent, rightPercent;

    private int frameWidth, frameHeight;
    private Mat currentInput, blurred;
    private Mat
            leftRegion, centerRegion, rightRegion,
            leftMasked, centerMasked, rightMasked;
    private Mat[] regions;
    private Mat[] masked;

    @Override
    public void init(int width, int height, CameraCalibration calibration){
        frameWidth = width;
        frameHeight = height;
        confidences = new double[] {0, 0, 0};
        regions = new Mat[] {null, null, null};
        masked = new Mat[] {null, null, null};
        regionBounds = new Rect[] {
                new Rect(
                        (int) (LEFT_TLCORNER_X * frameWidth), (int) (LEFT_TLCORNER_Y * frameHeight),
                        (int) (LEFT_WIDTH * frameWidth), (int) (LEFT_HEIGHT * frameHeight)
                ),
                new Rect(
                        (int) (CENTER_TLCORNER_X * frameWidth), (int) (CENTER_TLCORNER_Y * frameHeight),
                        (int) (CENTER_WIDTH * frameWidth), (int) (CENTER_HEIGHT * frameHeight)
                ),
                new Rect(
                        (int) (RIGHT_TLCORNER_X * frameWidth), (int) (RIGHT_TLCORNER_Y * frameHeight),
                        (int) (RIGHT_WIDTH * frameWidth), (int) (RIGHT_HEIGHT * frameHeight)
                ),
        };
        fullBounds = new Rect(0,0, frameWidth, frameHeight);
        regionSizes = new double[] { regionBounds[0].area(), regionBounds[1].area(), regionBounds[2].area() };
    }

    @Override
    public Mat processFrame(Mat input, long captureTimeNanos) {
        // Frame preprocessing: RGB to HSV + normalized Box Filter Blur
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV); // TODO: u sure its not R-G-B
        Imgproc.blur(input, blurred, new Size(frameWidth, frameHeight));
        blurred = blurred.submat(fullBounds);

        // Detect the prop within the regions
        for (int i = 0; i < 3; i++) {
            // crop out the relevant region & store
            regions[i] = blurred.submat( regionBounds[i] );

            // color mask it & store
            Core.inRange( regions[i], RED_LOWER_BOUNDS, RED_UPPER_BOUNDS, masked[i] );

            // get confidence percentage (proportion of pixels that were not masked out) & store
            confidences[i] = Core.countNonZero(masked[i]) / regionSizes[i];
        }
//        leftRegion = input.submat(leftBounds);
//        centerRegion = input.submat(centerBounds);
//        rightRegion = input.submat(rightBounds);
//
//        Core.inRange(leftRegion, RED_LOWER_BOUNDS, RED_UPPER_BOUNDS, leftMasked);
//        Core.inRange(centerRegion, RED_LOWER_BOUNDS, RED_UPPER_BOUNDS, centerMasked);
//        Core.inRange(rightRegion, RED_LOWER_BOUNDS, RED_UPPER_BOUNDS, rightMasked);
//
//        double total_pixels = frameWidth * frameHeight; // TODO: this isn't even right
//        leftPercent = Core.countNonZero(leftMasked) / total_pixels;
//        centerPercent = Core.countNonZero(centerMasked) / total_pixels;
//        rightPercent = Core.countNonZero(rightMasked) / total_pixels;

        double maxPercent = leftPercent == 0 && centerPercent == 0 && rightPercent == 0 ? -1 : Math.max(leftPercent, Math.max(centerPercent, rightPercent));

        detectionConfidence = maxPercent;

        if(maxPercent == leftPercent){
            detectedPosition = PropPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    new Point(0,0),
                    new Point(TeamVisionUtilities.CAMERA_RESOLUTION_WIDTH / 3d, TeamVisionUtilities.CAMERA_RESOLUTION_HEIGHT),
                    RED_LOWER_BOUNDS);
        }
        else if(maxPercent == centerPercent){
            detectedPosition = PropPosition.CENTER;
        }
        else if(maxPercent == rightPercent){
            detectedPosition = PropPosition.RIGHT;
        }
        else{
            detectedPosition = PropPosition.NONE;
        }


        currentInput = input;

        //stuff
        return blurred;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        
    }

    public PropPosition getPropPosition() {
        return detectedPosition;
    }

    public double getPropPercent(){
        return detectionConfidence;
    }

    public Mat getCurrentInput(){
        return currentInput;
    }

    public Mat getLeftRegion(){ return leftRegion; }
    public Mat getCenterRegion(){ return centerRegion; }
    public Mat getRightRegion(){ return rightRegion; }
    public Mat getBlurred(){ return blurred;}

    public Mat getLeftMasked() { return leftMasked; }
    public Mat getCenterMasked() { return centerMasked; }
    public Mat getRightMasked() { return rightMasked; }
}
