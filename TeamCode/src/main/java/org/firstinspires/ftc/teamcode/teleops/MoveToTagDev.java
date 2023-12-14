package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Octonaut;
import org.firstinspires.ftc.teamcode.util.vision.TeamVisionUtilities;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public class MoveToTagDev extends LinearOpMode {
    Octonaut robot;
    AprilTagProcessor tagProcessor;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(this);
        tagProcessor = TeamVisionUtilities.getAprilTagProcessor();

        waitForStart();
        while(opModeIsActive()) {

        }
    }
}
