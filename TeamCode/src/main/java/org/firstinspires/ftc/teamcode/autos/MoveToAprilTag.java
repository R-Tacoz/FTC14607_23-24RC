package org.firstinspires.ftc.teamcode.autos;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.robots.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.Octonaut;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robots.RobotBase;
import org.firstinspires.ftc.teamcode.util.odometry.Pose;
import org.firstinspires.ftc.teamcode.util.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.vision.TeamVisionUtilities;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.TimeUnit;
import java.util.stream.Collectors;
import java.util.stream.Stream;

// USE SCRCPY TO VIEW CAMERA STREAM DURING OPMODE

@Config
@Autonomous(name = "AprilTag Auto", group = "Main")
public class MoveToAprilTag extends LinearOpMode {
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    public static double exposureMS = 1;

    private static final Set<Integer>
            blueTagIds = new HashSet<>(Arrays.asList(1,2,3)),
            redTagIds = new HashSet<>(Arrays.asList(4,5,6));


    private static final Map<Integer, Pose2d> destinationOffests = Stream.of(new Object[][] {
            { 1, new Pose2d() },
            { 2, new Pose2d() },
            { 3, new Pose2d() },
            { 4, new Pose2d() },
            { 5, new Pose2d() },
            { 6, new Pose2d() },
    }).collect(Collectors.toMap(data -> (Integer) data[0], data -> (Pose2d) data[1]));

    private List<AprilTagDetection> detectedTags;

    MecanumDrive robot;

    @Override
    public void runOpMode(){
//        robot = new Octonaut(this);
        robot = new MecanumDrive(this, Octonaut.DIMENSIONS);

        initVision();
        telemetry.addLine("VisionPortal Initialized.");
        telemetry.update();
        sleep(1000);

        double dist = 0, strafe = 0;
        detectedTags = aprilTagProcessor.getDetections();
        telemetry.addData("# Detected Tags", detectedTags.size());
        telemetry.update();
        for(AprilTagDetection detectedTag : detectedTags) {
            if (detectedTag.metadata != null) {
                dist = detectedTag.ftcPose.range * Math.cos(Math.toRadians(detectedTag.ftcPose.bearing));
                strafe = detectedTag.ftcPose.range * Math.sin(Math.toRadians(detectedTag.ftcPose.bearing));
                telemetry.addData("Detected Tag", detectedTag.id);
                telemetry.addData("Distance (in)", dist);
                break;
            }
        }
        telemetry.addData("(Ticks)", robot.distanceToTicks((dist*2.54)-5));
        telemetry.update();


        waitForStart();
        robot.forward((dist*2.54)-20, 500);
        robot.left((strafe*2.54), 500);
//        sleep(500);
//        while(opModeIsActive()) {
//
//            detectedTags = aprilTagProcessor.getDetections();
//
//            Pose2d oldPos = new Pose2d();
//            for(AprilTagDetection detectedTag : detectedTags) {
//                if (detectedTag.metadata != null) {
//
//                    Pose2d destination = new Pose2d(
//                            detectedTag.ftcPose.range * Math.sin(Math.toRadians(detectedTag.ftcPose.bearing)),
//                            detectedTag.ftcPose.range * Math.cos(Math.toRadians(detectedTag.ftcPose.bearing)),
//                            Math.toRadians(detectedTag.ftcPose.yaw)
//                    );//.plus(destinationOffests.getOrDefault(detectedTag.id, new Pose2d()));
//
//                    telemetry.addData("Relative Dest", destination);
//
////                    destination.plus(oldPos);
//
////                    SampleMecanumDrive drive = null;
////                    // all in inches
////                    Pose2d origin = new Pose2d();
////                    Trajectory trajectory = drive.trajectoryBuilder(origin)
////                            .lineToLinearHeading(new Pose2d(detectedTag.center.x, detectedTag.center.y, Math.toRadians(90)).plus(origin))
////                            .build();
//
//
//                    telemetry.addData("Target" , "ID %d (%s)" , detectedTag.id, detectedTag.metadata.name);
//                    telemetry.addData("Range", "%5.1f inches", detectedTag.ftcPose.range);
//                    telemetry.addData("Pitch", "%5.0f degrees", detectedTag.ftcPose.pitch); //up down
//                    telemetry.addData("Yaw", "%5.0f degrees", detectedTag.ftcPose.yaw); //left and right
//                    telemetry.addData("Bearing" , "%5.0f degrees", detectedTag.ftcPose.bearing); //turn
//                    telemetry.addLine("--------------------------------------");
//                }
//            }
//            telemetry.update();
//        }
//
////        visionPortal.stopStreaming(); // conserve resources
        visionPortal.close(); // permanent stop
        sleep(500);
//        while (visionPortal.getCameraState() == VisionPortal.CameraState.CLOSING_CAMERA_DEVICE) {}
    }

    private void initVision() {
        aprilTagProcessor = TeamVisionUtilities.getAprilTagProcessor();
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTagProcessor)
                .enableLiveView(true) // TODO: what does this do
                .setCameraResolution(new Size(TeamVisionUtilities.CAMERA_RESOLUTION_WIDTH, TeamVisionUtilities.CAMERA_RESOLUTION_HEIGHT) )
                .build();

        // wait for the camera to begin streaming
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING))
                sleep(20);
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            // Set exposure.  Make sure we are in Manual Mode for these values to take effect.
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long) exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            telemetry.addData("Set Exposure", "Successful");
        } else {
            telemetry.addData("Set Exposure", "Failed");
        }
        telemetry.update();
    }
}
