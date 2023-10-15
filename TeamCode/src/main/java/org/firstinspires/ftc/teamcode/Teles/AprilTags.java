package org.firstinspires.ftc.teamcode.Teles;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Robots.ToBeNamed;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp (name = "AprilTag_Test", group = "Test")
public class AprilTags extends LinearOpMode {
    //CV
    private static final boolean USE_WEBCAM = true;
    private static final int RESOLUTION_WIDTH = 640;
    private static final int RESOLUTION_HEIGHT = 480;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private AprilTagDetection aprilTagDetections = null; // holds data for detected AprilTag

    @Override
    public void runOpMode(){
        initAprilTag();

        waitForStart();

        while(opModeIsActive()){

            List<AprilTagDetection> detectedTags = aprilTag.getDetections();

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


        }
    }

    private void initAprilTag() {
        //statically create & build aprilTag
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        if(USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT) )
                    .build();
        }else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .setCameraResolution(new Size(RESOLUTION_WIDTH, RESOLUTION_HEIGHT) )
                    .build();
        }
    }
}