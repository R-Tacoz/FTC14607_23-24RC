package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Octonaut;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.SpikeDetectPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;
@Config
@Autonomous(name = "EOCV Auto", group = "main")
public class SpikeMark extends LinearOpMode {
    Octonaut robot;
    OpenCvWebcam webcam;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(hardwareMap,this, 0,0, 8192, 5, 30, 0);

        // initialize webcam and detect color using pipeline
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        SpikeDetectPipeline pipeline = new SpikeDetectPipeline();
        webcam.setPipeline(pipeline);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
            }

        });
        // -----------------------------------------------------------------------------------------
        waitForStart();

        SpikeDetectPipeline.SpikePosition spike = pipeline.getSpikePosition();
        telemetry.addData("Detected position", spike);
        telemetry.update();
        sleep(  1000);

        int zone = 0;
        if (spike == SpikeDetectPipeline.SpikePosition.LEFT) {
            zone = 1;
        } else if (spike == SpikeDetectPipeline.SpikePosition.CENTER) {
            zone = 2;
        } else {
            zone = 3;
        }
        webcam.stopStreaming();
        webcam.stopRecordingPipeline();
        //Do moving stuff
    }
}
