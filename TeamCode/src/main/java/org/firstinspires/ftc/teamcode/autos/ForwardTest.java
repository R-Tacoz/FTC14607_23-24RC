package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robots.Inktonaut;

@Autonomous(name = "Test Auto", group = "Main")
public class ForwardTest extends LinearOpMode {
    Inktonaut robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Inktonaut(this);

        waitForStart();
        robot.forward(100, 400);

        robot.left(60, 400);

        robot.rotate(90);

    }
}
