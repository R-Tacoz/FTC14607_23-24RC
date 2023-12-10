package org.firstinspires.ftc.teamcode.util.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

/*
 * VisionPortal-based pipeline to detect the position and color of the team prop on the randomized
 * Spike Mark
 */
public class PropDetectProcessor implements VisionProcessor {

    public static final double DETECTION_THRESHOLD = 0.2;
    public static final int
            NO_DETECTION = -1,
            LEFT_DETECTION = 0,
            CENTER_DETECTION = 1,
            RIGHT_DETECTION = 2;
    public static final int
            RED_DETECTION = 0,
            BLUE_DETECTION = 1;
    private volatile int detectedPosition = NO_DETECTION;
    private volatile int detectedColor = RED_DETECTION;
    private volatile double detectionConfidence = 0;

    // Color boundaries for masking. Adjust as needed. In BGR (not RGB)
    private static final Scalar
            RED = new Scalar(0, 0, 255),
            BLUE = new Scalar(255, 0, 0),
            RED_LOWER_BOUNDS =  new Scalar(0  , 255, 100),
            RED_UPPER_BOUNDS =  new Scalar(176 , 224 , 255), // TODO: red improper clipping
            BLUE_LOWER_BOUNDS = new Scalar(120, 255, 100),
            BLUE_UPPER_BOUNDS = new Scalar(114, 192, 255),
            RED_LOWER_LBOUNDS =  new Scalar(0  , 224, 100),
            RED_UPPER_LBOUNDS =  new Scalar(15 , 255 , 255),
            RED_LOWER_RBOUNDS =  new Scalar(165  , 224, 100),
            RED_UPPER_RBOUNDS =  new Scalar(179 , 255 , 255);
//            BLUE_LOWER_BOUNDS = new Scalar(80, 152, 100),
//            BLUE_UPPER_BOUNDS = new Scalar(180, 255, 255);
    // vvv in BGR vvv
//            RED_LOWER_BOUNDS =  new Scalar(0  , 0  , 100),
//            RED_UPPER_BOUNDS =  new Scalar(63 , 31 , 255),
//            BLUE_LOWER_BOUNDS = new Scalar(100, 0  , 0),
//            BLUE_UPPER_BOUNDS = new Scalar(255, 100, 63);
    // old
//            RED_LOWER_BOUNDS = new Scalar(155, 55, 55),
//            RED_UPPER_BOUNDS = new Scalar(179, 225, 255),
//            BLUE_LOWER_BOUNDS = new Scalar(100, 0, 0),
//            BLUE_UPPER_BOUNDS = new Scalar(255,50,50);


    // Regions to color mask; values are proportions of the screen. Adjust as needed.
    private static final double
            LEFT_TLCORNER_X     = 0,    LEFT_TLCORNER_Y     = 0,
            LEFT_WIDTH          = 0.33, LEFT_HEIGHT         = 1,

            CENTER_TLCORNER_X   = 0.33, CENTER_TLCORNER_Y   = 0,
            CENTER_WIDTH        = 0.33, CENTER_HEIGHT       = 1,

            RIGHT_TLCORNER_X    = 0.66, RIGHT_TLCORNER_Y    = 0,
            RIGHT_WIDTH         = 0.33, RIGHT_HEIGHT        = 1;

    private Rect fullBounds;
    private Rect[] regionBounds;
    private double[] regionSizes; // # pixels in each region
    private final double[][] confidences = new double[2][3];

    private int frameWidth, frameHeight;
    private volatile Mat currentInput, blurred;
    private final Mat[] regions = new Mat[3]; // portions of the frame
    private final Mat[][] masked = new Mat[2][3]; // regions, color masked

    @Override
    public void init(int width, int height, CameraCalibration calibration){
        frameWidth = width;
        frameHeight = height;
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
        currentInput = input;
        Imgproc.cvtColor(input, input, Imgproc.COLOR_BGR2HSV); // TODO: try without convert
        Imgproc.blur(input, blurred, new Size(frameWidth, frameHeight));
        blurred = blurred.submat(fullBounds);

        // Detect the prop within the regions
        for (int i = 0; i < 3; i++) {
            // crop out the relevant region & store
            regions[i] = blurred.submat( regionBounds[i] );

            // color mask it & store
            Mat redMaskedL = new Mat(), redMaskedR = new Mat();
            Core.inRange( regions[i], RED_LOWER_BOUNDS, RED_UPPER_BOUNDS, masked[0][i] );
//            Core.inRange( regions[i], RED_LOWER_LBOUNDS, RED_UPPER_LBOUNDS, redMaskedL );
//            Core.inRange( regions[i], RED_LOWER_RBOUNDS, RED_UPPER_RBOUNDS, redMaskedR );
//            Core.add(redMaskedL, redMaskedR, masked[0][i]);

            Core.inRange( regions[i], BLUE_LOWER_BOUNDS, BLUE_UPPER_BOUNDS, masked[1][i] );

            // get confidence percentage (proportion of pixels that were not masked out) & store
            confidences[0][i] = Core.countNonZero(masked[0][i]) / regionSizes[i];
            confidences[1][i] = Core.countNonZero((masked[1][i])) / regionSizes[i];
        }

        // get the most confident detection
        double maxConfidence = 0;
        int maxColor = -1, maxPos = -1;
        for (int color = 0; color < 1; color++) {
            for (int pos = 0; pos < 2; pos ++) {
                if (confidences[color][pos] > maxConfidence) {
                    maxConfidence = confidences[color][pos];
                    maxColor = color;
                    maxPos = pos;
                }
            }
        }

        detectedColor = maxColor;
        detectedPosition = maxPos;
        detectionConfidence = maxConfidence;
        if (detectionConfidence < DETECTION_THRESHOLD) {
            detectedPosition = NO_DETECTION;
            return blurred;
        } else {
            Scalar color = (detectedColor == 0) ? RED:BLUE;
            Imgproc.rectangle(input, regionBounds[detectedPosition], color);
        }

        return blurred;
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        canvas.drawRect(0, 0, 100, 100, new Paint(Color.MAGENTA));
    }

    public int getDetectedPosition() {
        return detectedPosition;
    }
    public int getDetectedColor() {
        return detectedColor;
    }
    public double getDetectionConfidence(){
        return detectionConfidence;
    }
    public Mat getCurrentInput(){
        return currentInput;
    }
    public Mat getBlurred(){
        return blurred;
    }

}
