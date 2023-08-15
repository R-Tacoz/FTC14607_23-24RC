package org.firstinspires.ftc.teamcode.Teles;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robots.RobotBase;
import org.firstinspires.ftc.teamcode.Robots.ToBeNamed;

@TeleOp(name = "Main TeleOp", group = "main")
public class IdiotsInCars extends LinearOpMode {
    // Finite States
    private enum OpState { START, }

    // Etc
    ToBeNamed robot;

    public void runOpMode() {
        robot = new ToBeNamed(this, 0,0, 30, 0);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("imu", robot.imu.getAngularOrientation());
            telemetry.addData("context", robot.odometry.getContext());
            telemetry.update();
        }
    }
}
