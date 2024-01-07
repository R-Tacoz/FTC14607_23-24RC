package org.firstinspires.ftc.teamcode.util.vision;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public final class TeamVisionUtilities {

    // Microsoft LifeCam Cinema HD USB Webcam
    public static int CAMERA_RESOLUTION_WIDTH = 640;
    public static int CAMERA_RESOLUTION_HEIGHT = 480;
    public static long CAMERA_EXPOSURE_MS = 1;
    // * From 3DF Zephyr. Instructions: https://ftc-docs.firstinspires.org/en/latest/programming_resources/vision/camera_calibration/camera-calibration.html
    public static double
            INTRINSICS_FX = 594.974989979,
            INSTRINSICS_FY = 594.974989979,
            INSTRINSICS_CX = 326.694716194,
            INTRINSICS_CY = 217.453950995;

    private TeamVisionUtilities() {} // disallow instantiation bc u shouldnt be

    public static AprilTagProcessor getAprilTagProcessor(){
//
//        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
//                .setAllowOverwrite(true)
//                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .addTag(10, "temp", 5, DistanceUnit.CM) // TODO: vectorfield calculations
//                .build();

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                .setLensIntrinsics(INTRINSICS_FX, INSTRINSICS_FY, INSTRINSICS_CX, INTRINSICS_CY)
                .build();

        return aprilTagProcessor;
    }
//
//    public static void initVision(LinearOpMode opmode, VisionPortal visionPortal, AprilTagProcessor aprilTagProcessor) {
//        aprilTagProcessor = TeamVisionUtilities.getAprilTagProcessor();
//        visionPortal = new VisionPortal.Builder()
//                .setCamera(opmode.hardwareMap.get(WebcamName.class, "Webcam 1"))
//                .addProcessor(aprilTagProcessor)
//                .enableLiveView(true) // TODO: what does this do
//                .setCameraResolution(new Size(CAMERA_RESOLUTION_WIDTH, CAMERA_RESOLUTION_HEIGHT) )
//                .build();
//
//        // wait for the camera to begin streaming
//        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
//            opmode.telemetry.addData("Camera", "Waiting");
//            opmode.telemetry.update();
//            while (!opmode.isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING))
//                opmode.sleep(20);
//            opmode.telemetry.addData("Camera", "Ready");
//            opmode.telemetry.update();
//        }
//
//        // Set camera controls unless we are stopping.
//        if (!opmode.isStopRequested())
//        {
//            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
//            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
//            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
//                exposureControl.setMode(ExposureControl.Mode.Manual);
//                opmode.sleep(50);
//            }
//            exposureControl.setExposure(CAMERA_EXPOSURE_MS, TimeUnit.MILLISECONDS);
//            opmode.sleep(20);
//            opmode.telemetry.addData("Set Exposure", "Successful");
//        } else {
//            opmode.telemetry.addData("Set Exposure", "Failed");
//        }
//        opmode.telemetry.update();
//    }

}
