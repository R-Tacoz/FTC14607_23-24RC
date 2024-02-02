package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Octonaut;
import org.firstinspires.ftc.teamcode.util.vision.PropDetectProcessor;
import org.firstinspires.ftc.vision.VisionProcessor;

public class SpikeMark {
    public PropDetectProcessor spikeProcessor = null;
    private LinearOpMode opmode = null;
    private Octonaut robot = null;
    private int detectedPos = PropDetectProcessor.NO_DETECTION;
    private int detectedColor;

    public SpikeMark(LinearOpMode opmode, Octonaut robot){
        this.opmode = opmode;
        this.robot = robot;
    }
    public void init(){
        spikeProcessor = new PropDetectProcessor();
    }
    public VisionProcessor getProcessor(){
        return this.spikeProcessor;
    }
    public void run() {
        while (opmode.opModeIsActive() && detectedPos == PropDetectProcessor.NO_DETECTION) {
            onUpdate();
        }
        moveToProp();
    }
    private void onUpdate(){
        this.detectedPos = spikeProcessor.getDetectedPosition();
        this.detectedColor = spikeProcessor.getDetectedColor();
    }
    private void moveToProp(){
        this.robot.forward(56, 200); //test
        if(detectedPos == PropDetectProcessor.LEFT_DETECTION){
            this.robot.rotate(-90);
            this.robot.intake();
            this.robot.rotate(90);
        }
        else if(detectedPos == PropDetectProcessor.CENTER_DETECTION){
            this.robot.forward(10, 200);
            this.robot.intake();
        }
        else{
            this.robot.rotate(90);
            this.robot.intake();
            this.robot.rotate(-90);
        }

        if(this.detectedColor == PropDetectProcessor.RED_DETECTION){
            this.robot.rotate(-90);
        }
        else{
            this.robot.rotate(90);
        }

        //At the end of SpikeMark, the robot will be facing the board
    }

}
