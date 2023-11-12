package org.firstinspires.ftc.teamcode.Teles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Robots.ToBeNamed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp (name = "AprilTag_Test", group = "Test")
public class AprilTags extends LinearOpMode {
    //CV
    private static final int RESOLUTION_WIDTH = 640; // TODO: fiddle resolution
    private static final int RESOLUTION_HEIGHT = 480;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;
    private AprilTagDetection aprilTagDetections = null; // holds data for detected AprilTag

    @Override
    public void runOpMode(){
        initAprilTag();
        telemetry.addLine("VisionPortal Initialized.");
        telemetry.update();

        waitForStart();

        while(opModeIsActive()){

            List<AprilTagDetection> detectedTags = aprilTagProcessor.getDetections();

            for(AprilTagDetection detectedTag : detectedTags) {
                if (detectedTag.metadata != null) {
                    telemetry.addData("Target" , "ID %d (%s)" , detectedTag.id, detectedTag.metadata.name);
                    telemetry.addData("Range", "%5.1f inches", detectedTag.ftcPose.range);
                    telemetry.addData("Pitch", "%5.0f degrees", detectedTag.ftcPose.pitch); //up down
                    telemetry.addData("Yaw", "%5.0f degrees", detectedTag.ftcPose.yaw); //left and right
                    telemetry.addData("Bearing" , "%5.0f degrees", detectedTag.ftcPose.bearing); //turn
                    telemetry.addLine("--------------------------------------");
                }
            }
            telemetry.update();


            float fps = visionPortal.getFps(); // get FPS
        }
        visionPortal.stopStreaming(); // conserve resources
        visionPortal.resumeStreaming();
        visionPortal.close(); // permanent stop
    }

    private void initAprilTag() {
        //statically create & build aprilTag
        AprilTagLibrary tagLibrary = new AprilTagLibrary.Builder()
                .setAllowOverwrite(true)
                .addTags(AprilTagGameDatabase.getCurrentGameTagLibrary())
                .addTag(10, "temp", 5, DistanceUnit.CM) // TODO: vectorfield calculations
                .build();
        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setTagLibrary(tagLibrary)
                .setLensIntrinsics(594.974989979, 594.974989979, 326.694716194, 217.453950995) // from 3DF Zephyr
                .build();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(false)
                .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT) )
                .build();
    }
}