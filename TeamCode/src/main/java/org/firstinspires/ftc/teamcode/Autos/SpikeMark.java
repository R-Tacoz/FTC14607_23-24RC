package org.firstinspires.ftc.teamcode.Autos;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Robots.ToBeNamed;

public class SpikeMark extends LinearOpMode {
    ToBeNamed robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new ToBeNamed(this, -1,-1, -1,-1,-1,-1);


        // -----------------------------------------------------------------------------------------
        waitForStart();
    }
}
