package org.firstinspires.ftc.teamcode.Robots;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Odometry;
import org.firstinspires.ftc.teamcode.util.ThreeWheelOdometry;

public class Octonaut extends RobotBase {
    public Odometry odometry;
    public int odoTicks;
    public Servo tester;

    public Octonaut(
        LinearOpMode opModeInstance,
        int drivetrainTicks,
        int wheelDiameter,
        int odoTicks,
        float odoWheelDiam,
        float odoWidth,
        float odoBackDist
    ) {
        super(opModeInstance, drivetrainTicks, wheelDiameter);
        odometry = new ThreeWheelOdometry(
            hardwareMap.get(DcMotor.class, "odoRight"),
            hardwareMap.get(DcMotor.class, "odoLeft"),
            hardwareMap.get(DcMotor.class, "odoBack"),
            odoTicks,
            odoWheelDiam,
            odoWidth,
            odoBackDist
        );

    }
    public void testingServo() {
        this.tester.setPosition(0);
    }
}
