package org.firstinspires.ftc.teamcode.autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.util.vision.SpikeDetectProcessorRed;
import org.firstinspires.ftc.teamcode.robots.Octonaut;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.vision.UtilityCameraFrameCapture;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "Main Auto", group = "Main")
public class UniGarten extends LinearOpMode {
    Octonaut robot;
    VisionPortal visionportal;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(this);


    }
}
