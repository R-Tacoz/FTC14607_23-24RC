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

import org.firstinspires.ftc.teamcode.robots.Inktonaut;
import org.firstinspires.ftc.teamcode.robots.Octonaut;

@TeleOp(name = "Main Linear TeleOp", group = "Main")
public class Gaybians extends LinearOpMode {

    // Etc
    Inktonaut robot;
    public final DecimalFormat fourDecimals = new DecimalFormat("#.0000");
    public final float slowSpeed = 0.3f;
    public final float fastSpeed = 0.54f;
    public boolean reverseDrivetrain = false;
    public boolean movingDrivetrain = false;
    public float lastY = 0;
    public float lastX = 0;
    public float lastRX = 0;

    private boolean fastSlides = true;
    private boolean movingSlide = false;
    private byte slideDirection = 1; // direction the slides were just previously moving -1 is down, 1 is up, 0 is for set heights
    private int lastSlidePos = 0;
    private boolean movingToGround = false;

    /**
     * Moves the drivetrain (the four wheels). Includes strafing and rotation
     * @param gamepad Gamepad that controls the drivetrain (gamepad1 or gamepad2)
     * @param speedFactor Factor which the inputs are multiplied
     * @return driveTrainPowers, an array of all the powers the drivetrain is set to (FL, BL, FR, BR)
     */
    public float[] moveDriveTrain(@NonNull Gamepad gamepad, @NonNegative float speedFactor) {
        //control.drivetrain represents all 4 wheels below?
        //control.frontLEft, backLeft, frontRight, backRight ... represent
        float fbinput = gamepad.left_stick_y; //either input is analog of digital based on which piece of gamepad is pressed
        if(gamepad.dpad_up) fbinput = 1;
        else if(gamepad.dpad_down) fbinput = -1;

        float y = -fbinput * speedFactor;
        float x = gamepad.left_stick_x * speedFactor * 1.75f;
        float rx = gamepad.right_stick_x * speedFactor * 0.6f;

        // brake using setVelocity
        if(y == 0 && x == 0 && rx == 0) {
            for (DcMotorEx m: robot.drivetrain) {
                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                m.setVelocity(0);
            }
            return new float[]{0,0,0,0};
        } else {
            for (DcMotorEx m : robot.drivetrain) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            float frontLeftPower = (y + x + rx);
            float backLeftPower = (y - x + rx) * 1.17f;
            float frontRightPower = (y - x - rx);
            float backRightPower = (y + x - rx) * 1.15f;
            boolean braking = false;

            float[] driveTrainPowers = new float[]{
                    Float.parseFloat(fourDecimals.format((double) frontLeftPower)),
                    Float.parseFloat(fourDecimals.format((double) backLeftPower)),
                    Float.parseFloat(fourDecimals.format((double) frontRightPower)),
                    Float.parseFloat(fourDecimals.format((double) backRightPower)),
            };
            robot.frontLeft.setPower(frontLeftPower);
            robot.backLeft.setPower(backLeftPower);
            robot.frontRight.setPower(frontRightPower);
            robot.backRight.setPower(backRightPower);

            return driveTrainPowers;
        }
    }

    /**
     * Moves the slides
     * @param gamepad Gamepad that controls the slides (gamepad1 or gamepad2)
     * @return The position of the slide in ticks ~[0,960]
     */
    public int moveSlides(@NonNull Gamepad gamepad) {
        int slidePos = robot.getSlidePos();
        double up = -gamepad.right_stick_y;
        double down = gamepad.right_stick_y;

        if(gamepad.dpad_up) fastSlides = true;
        else if(gamepad.dpad_down) fastSlides = false;
        int SLIDE_UP_VELO = (int)(1000 * (fastSlides ? 1:0.6));
        int SLIDE_DOWN_VELO = (int)(-800 * (fastSlides ? 1:0.45));
        // joystick: continuous
        if (up>0 || down>0) {
            if(up>0 && slidePos >= Octonaut.SLIDETOP-30) robot.setSlidePos(Octonaut.SLIDETOP); //buffer for the top
            else if(down>0 && slidePos <= Octonaut.SLIDEBOTTOM+30) robot.setSlidePos(Octonaut.SLIDEBOTTOM); //buffer for bottom
            else {
                movingSlide = true;
                double velo = 0;
                if (up > 0) { // ticks/s
                    velo = SLIDE_UP_VELO; slideDirection = 1; }
                else {
                    velo = SLIDE_DOWN_VELO; slideDirection = -1; }
                robot.setSlideVelocity(velo);
            }
            // a,b,x,y, preset positions or hold current position
        }
        else {
            if (movingSlide) {
                movingSlide = false;
                lastSlidePos = slidePos + 5*slideDirection; //essentially stops moving the slides when you let go of the vertical stick
            }
            int setPos = lastSlidePos;
            slideDirection = 0;
            movingSlide = true;
            if (gamepad.y) {setPos = 300;}
            else if (gamepad.b) {setPos = 700;}
            else if (gamepad.a || movingToGround) {setPos = Octonaut.GROUND; }
            else {movingSlide = false; }
            // stop moving to ground if reached it
            if (slidePos < 20) movingToGround = false;
            setPos = Range.clip(setPos, Octonaut.SLIDEBOTTOM, Octonaut.SLIDETOP);
            robot.setSlidePos(setPos);
        }
        return slidePos;
    }

    //test servo (11/30/2023)
    private double servoPow = 1;
    private double servoPos = 0.666;
//    public void moveClaw(@NonNull Gamepad gamepad) {
//        double setServoPow = gamepad.left_bumper || gamepad.right_bumper ? servoPow : 0;
//
//        telemetry.addLine("Is it working");
//
//        if(gamepad.left_bumper) {
//            robot.outtake();
//            setServoPow = 0.2;
//        }
//        else if(gamepad.right_bumper) robot.intake();
//        robot.setClawPower(setServoPow);
//    }

    private double liftPos = 0;
//    public void moveLift(@NonNull Gamepad gamepad) {
//        liftPos = robot.lift.getPosition();
//        if (gamepad.right_trigger > 0) robot.setLift(liftPos + 0.02);
//        else if (gamepad.left_trigger > 0) robot.setLift(liftPos - 0.02);
//    }

    public void runOpMode() {
        robot = new Inktonaut(this);
        float speedFactor = 0.8f;

        for(DcMotorEx m:robot.drivetrain) m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();
        while(opModeIsActive())
        {
            telemetry.addData("imu", robot.imu.getAngularOrientation());
//            telemetry.addData("context", robot.odometry.getContext());
            float[] driveTrainPowers = moveDriveTrain(gamepad1, speedFactor);
//            moveClaw(gamepad2);
            int slidePos = moveSlides(gamepad2);
//            moveLift(gamepad2);

            telemetry.addData("Speed factor", speedFactor);
            telemetry.addData("Drivetrain powers", Arrays.toString(driveTrainPowers));
            telemetry.addData("Servo power: ", servoPow);
            telemetry.addData("Slide position", slidePos);
            telemetry.addData("Slide speed", fastSlides);
            telemetry.addData("Lift position", liftPos);
            telemetry.update();
        }
    }
}
