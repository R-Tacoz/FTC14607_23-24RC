package org.firstinspires.ftc.teamcode.Autos;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.CV.SpikeDetectProcessorBlue;
import org.firstinspires.ftc.teamcode.Robots.Octonaut;
import org.firstinspires.ftc.teamcode.util.UtilityCameraFrameCapture;
import org.firstinspires.ftc.vision.VisionPortal;

@Config
@Autonomous(name = "EOCV Auto - Right Blue", group = "main")
public class SpikeMarkBlue extends LinearOpMode {
    Octonaut robot;
    VisionPortal visionportal;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(hardwareMap,this, 0,10, 8192, 5, 30, 0);

        SpikeDetectProcessorBlue spikeProcessor = new SpikeDetectProcessorBlue();

        visionportal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(UtilityCameraFrameCapture.RESOLUTION_WIDTH, UtilityCameraFrameCapture.RESOLUTION_HEIGHT))
                .addProcessor(spikeProcessor)
                .enableLiveView(true)
                .build();

        waitForStart();

//        while(opModeIsActive()){
        SpikeDetectProcessorBlue.SpikePosition spike = spikeProcessor.getSpikePosition();
        double percent = spikeProcessor.getSpikePercent();

        telemetry.addData("Detected position", spike);
        telemetry.addData("Detected Percentage", percent);
        telemetry.addLine(spikeProcessor.getCurrentInput().toString());
        telemetry.addData("leftmat ele" , spikeProcessor.getLeftMat());
        telemetry.addData("leftmatpurple", spikeProcessor.getLeftMatPurple());
        telemetry.addData("leftmat", spikeProcessor.getLeftMat());
        telemetry.update();

        //zones
        int zone = 0;
        if (spike == SpikeDetectProcessorBlue.SpikePosition.LEFT) {
            zone = 1;
        } else if (spike == SpikeDetectProcessorBlue.SpikePosition.CENTER) {
            zone = 2;
        } else {
            zone = 3;
        }
        visionportal.close();

        // go until spike mark panel
        robot.forward(56, 200);
        sleep(500);

        telemetry.addLine("froward");
        telemetry.update();

        //diff tasks depending on zone
        switch(zone) {
            case 2: // center
                break;
            case 3:
                robot.rotate(-90);
                break;
            case 1:
                robot.rotate(90);
                break;
        }

        // pray we got the right spot
        robot.outtake();
        robot.setClawPower(0.2);
        sleep(1000);

        // turn to the boards
        switch(zone) {
            case 2: // center
                robot.rotate(-90);
                break;
            case 3:
                break;
            case 1:
                robot.rotate(180);
                break;
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
