package org.firstinspires.ftc.teamcode.Robots;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Teles.IdiotsInCars;
import org.firstinspires.ftc.teamcode.util.Odometry;
import org.firstinspires.ftc.teamcode.util.ThreeWheelOdometry;

public class Octonaut extends RobotBase {
    public Odometry odometry;
    public int odoTicks;
    public DcMotorSimple tester;
    public Servo tester2;

    public Octonaut(@NonNull HardwareMap hardwareMap,
        LinearOpMode opModeInstance,
        int drivetrainTicks,
        int wheelDiameter,
        int odoTicks,
        float odoWheelDiam,
        float odoWidth,
        float odoBackDist
    ) {
        super(hardwareMap, opModeInstance, drivetrainTicks, wheelDiameter);
//        odometry = new ThreeWheelOdometry(
//            hardwareMap.get(DcMotor.class, "odoRight"),
//            hardwareMap.get(DcMotor.class, "odoLeft"),
//            hardwareMap.get(DcMotor.class, "odoBack"),
//            odoTicks,
//            odoWheelDiam,
//            odoWidth,
//            odoBackDist
//        );

        tester = hardwareMap.get(DcMotorSimple.class, "tester");
        tester2 = hardwareMap.get(Servo.class, "tester2");

    }

    public void setClawPower(double pow){
        tester.setPower(pow);
    }
    public void setClawPos(double pos){
        double position = Range.clip(pos, -1, 1);
        tester.setPower(position);
    }
}
