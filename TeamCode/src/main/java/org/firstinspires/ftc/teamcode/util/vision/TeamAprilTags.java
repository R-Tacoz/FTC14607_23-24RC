package org.firstinspires.ftc.teamcode.util.vision;

import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public final class TeamAprilTags {

    private TeamAprilTags() {}

    public static AprilTagProcessor getAprilTagProcessor(){
//
//        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
//                .setAllowOverwrite(true)
//                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
//                .addTag(10, "temp", 5, DistanceUnit.CM) // TODO: vectorfield calculations
//                .build();

        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                // params for Logitech XXXX from 3DF Zephyr
                .setLensIntrinsics(594.974989979, 594.974989979, 326.694716194, 217.453950995)
                .build();

        return aprilTagProcessor;
    }

}
