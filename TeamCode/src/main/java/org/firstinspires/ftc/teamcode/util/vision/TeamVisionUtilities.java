package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public final class TeamVisionUtilities {

    // Microsoft LifeCam Cinema HD USB Webcam
    public static int CAMERA_RESOLUTION_WIDTH = 640;
    public static int CAMERA_RESOLUTION_HEIGHT = 480;
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

}
