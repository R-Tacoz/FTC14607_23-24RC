package org.firstinspires.ftc.teamcode.autos;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.robots.Inktonaut;
import org.firstinspires.ftc.teamcode.robots.Octonaut;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.odometry.Pose;
import org.firstinspires.ftc.teamcode.util.roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.vision.PropDetectProcessor;
import org.firstinspires.ftc.teamcode.util.vision.TeamVisionUtilities;
import org.firstinspires.ftc.teamcode.util.vision.UtilityCameraFrameCapture;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagMetadata;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@Config
@Autonomous(name = "Main Auto", group = "Main")
public class UniGarten extends LinearOpMode {
    Inktonaut robot = null;

    @Override
    public void runOpMode() throws InterruptedException {
        //Park
        robot = new Inktonaut(this);
        waitForStart();

        while(opModeIsActive()){

        }
    }
}
