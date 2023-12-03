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
import org.opencv.core.Mat;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous(name = "EOCV Auto - Right Blue (Left Red)", group = "main")
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

        SpikeDetectProcessor.SpikePosition spike = SpikeDetectProcessor.SpikePosition.NONE;

        while(spike == SpikeDetectProcessor.SpikePosition.NONE){
            spike = spikeProcessor.getSpikePosition();
        }

        visionportal.close();

        telemetry.addData("Final Spike", spike);
        telemetry.update();

        // go until spike mark panel
        robot.forward(56, 200);
        sleep(500);

        if(spike == SpikeDetectProcessor.SpikePosition.LEFT){
            robot.rotate(-90);
        }
        else if(spike == SpikeDetectProcessor.SpikePosition.RIGHT){
            robot.rotate(90);
        }

        //move to board
        robot.forward(130, 200);

        //at board and want to deposit
        robot.setSlidePos(300);
        sleep(500);
        robot.outtake();
        robot.setClawPower(0.2);
        sleep(1000);
        robot.setSlidePos(Octonaut.GROUND);
        sleep(1000);
    }
}

