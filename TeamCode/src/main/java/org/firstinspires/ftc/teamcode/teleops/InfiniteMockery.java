package org.firstinspires.ftc.teamcode.teleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robots.Inktonaut;

import java.util.Arrays;

@Disabled
@TeleOp(name = "Main FSM TeleOp", group = "Main")
public class InfiniteMockery extends LinearOpMode {

    Inktonaut robot;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Inktonaut(this);


        waitForStart();
        while(opModeIsActive()) {


            telemetry.addData("Drivetrain powers", 69);
            telemetry.update();
        }
    }

}
