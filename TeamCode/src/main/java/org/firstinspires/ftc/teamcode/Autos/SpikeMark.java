package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.CV.SpikeDetectProcessor;
import org.firstinspires.ftc.teamcode.Robots.Octonaut;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.UtilityCameraFrameCapture;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous(name = "EOCV Auto", group = "main")
public class SpikeMark extends LinearOpMode {
    Octonaut robot;
    VisionPortal visionportal;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(hardwareMap,this, 0,0, 8192, 5, 30, 0);

        SpikeDetectProcessor spikeProcessor = new SpikeDetectProcessor();

        visionportal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT))
                .addProcessor(spikeProcessor)
                .enableLiveView(true)
                .build();

        waitForStart();

        while(opModeIsActive()){
            SpikeDetectProcessor.SpikePosition spike = spikeProcessor.getSpikePosition();
            double percent = spikeProcessor.getSpikePercent();
            telemetry.addData("Detected position", spike);
            telemetry.addData("Detected Percentage", percent);
            telemetry.addLine(spikeProcessor.getCurrentInput().toString());
            telemetry.update();
        }

//        int zone = 0;
//        if (spike == SpikeDetectProcessor.SpikePosition.LEFT) {
//            zone = 1;
//        } else if (spike == SpikeDetectProcessor.SpikePosition.CENTER) {
//            zone = 2;
//        } else {
//            zone = 3;
//        }
//        //Do moving stuff


    }
}
