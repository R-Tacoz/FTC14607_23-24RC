package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.Octonaut;

public class SpikeMark extends LinearOpMode {
    Octonaut robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Octonaut(this, -1,-1, -1,-1,-1,-1);


        // -----------------------------------------------------------------------------------------
        waitForStart();
    }
}
