package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public final class VisionUtil {

    private VisionUtil() {}

    public static AprilTagProcessor getAprilTagProcessor(){
        //statically create & build aprilTag
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .setAllowOverwrite(true)
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .addTag(10, "temp", 5, DistanceUnit.CM) // TODO: vectorfield calculations
                .build();
        AprilTagProcessor aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(594.974989979, 594.974989979, 326.694716194, 217.453950995) // from 3DF Zephyr
                .build();
        return aprilTagProcessor;
    }

}
