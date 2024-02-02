package org.firstinspires.ftc.teamcode.teleops;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.checkerframework.checker.index.qual.NonNegative;
import java.text.DecimalFormat;
import java.util.Arrays;

import org.firstinspires.ftc.teamcode.robots.MecanumDrive;
import org.firstinspires.ftc.teamcode.robots.Octonaut;

@TeleOp(name = "Drivetrain Only", group = "Main")
public class DrivetrainOnly extends LinearOpMode {

    MecanumDrive robot;
    public final DecimalFormat fourDecimals = new DecimalFormat("#.0000");

    public float[] moveDriveTrain(@NonNull Gamepad gamepad, @NonNegative float speedFactor) {
        float FBinput = gamepad.left_stick_y;
        if(gamepad.dpad_up) FBinput = 1;
        else if(gamepad.dpad_down) FBinput = -1;

        float y = -FBinput * speedFactor;
        float x = gamepad.left_stick_x * speedFactor;
        float rx = gamepad.right_stick_x * speedFactor;

        for (DcMotorEx m : robot.drivetrain) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        float frontLeftPower = (y + x + rx);
        float backLeftPower = (y - x + rx) * 1.1f;
        float frontRightPower = (y - x - rx);
        float backRightPower = (y + x - rx) * 1.1f;

        float[] driveTrainPowers = new float[]{
                Float.parseFloat(fourDecimals.format(frontLeftPower)),
                Float.parseFloat(fourDecimals.format(backLeftPower)),
                Float.parseFloat(fourDecimals.format(frontRightPower)),
                Float.parseFloat(fourDecimals.format(backRightPower)),
        };

        robot.frontLeft.setPower(frontLeftPower);
        robot.backLeft.setPower(backLeftPower);
        robot.frontRight.setPower(frontRightPower);
        robot.backRight.setPower(backRightPower);

        return driveTrainPowers;
    }

    public void runOpMode() {
        robot = new MecanumDrive(this);
        float speedFactor = 1f;

        for(DcMotorEx m:robot.drivetrain) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("imu", robot.imu.getAngularOrientation());
//            telemetry.addData("context", robot.odometry.getContext());
            float[] driveTrainPowers = moveDriveTrain(gamepad1, speedFactor);

            telemetry.addData("Speed factor", speedFactor);
            telemetry.addData("Drivetrain powers", Arrays.toString(driveTrainPowers));
            telemetry.update();
        }
    }
}
