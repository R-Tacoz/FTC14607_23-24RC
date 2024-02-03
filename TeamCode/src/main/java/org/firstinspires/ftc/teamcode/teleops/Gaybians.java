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

    private boolean fastSlides = true;
    private boolean movingSlide = false;
    private byte slideDirection = 1; // direction the slides were just previously moving -1 is down, 1 is up, 0 is for set heights
    private int lastSlidePos = 0;
    private boolean movingToGround = false;
    public final double LOOP_SPEED = 0.020; // rough loop duration in seconds

    /**
     * Moves the drivetrain (the four wheels). Includes strafing and rotation
     * @param gamepad Gamepad that controls the drivetrain (gamepad1 or gamepad2)
     * @param speedFactor Factor which the inputs are multiplied
     * @return driveTrainPowers, an array of all the powers the drivetrain is set to (FL, BL, FR, BR)
     */
    public float[] moveDriveTrain(@NonNull Gamepad gamepad, @NonNegative float speedFactor) {
        //control.drivetrain represents all 4 wheels below?
        //control.frontLEft, backLeft, frontRight, backRight ... represent
        float FBInput = gamepad.left_stick_y; //either input is analog of digital based on which piece of gamepad is pressed
        if(gamepad.dpad_up) FBInput = -1;
        else if(gamepad.dpad_down) FBInput = 1;

        float y = -FBInput * speedFactor;
        float x = gamepad.left_stick_x * speedFactor * 1f;
        float rx = gamepad.right_stick_x * speedFactor * 1f;

        // brake using setVelocity
//        if(y == 0 && x == 0 && rx == 0) {
//            for (DcMotorEx m: robot.drivetrain) {
//                m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                m.setVelocity(0);
//            }
//            return new float[]{0,0,0,0};
//        } else {
            for (DcMotorEx m : robot.drivetrain) m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            float frontLeftPower = (y + x + rx);
            float backLeftPower = (y - x + rx);
            float frontRightPower = (y - x - rx);
            float backRightPower = (y + x - rx);

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
//        }
    }

    /**
     * Moves the slides
     * @param gamepad Gamepad that controls the slides (gamepad1 or gamepad2)
     * @return The position of the slide in ticks ~[0,960]
     */
    public int moveSlides(@NonNull Gamepad gamepad) {

        boolean goingToGround = gamepad.a;
        boolean goingUp = gamepad.dpad_up;
        boolean goingDown = gamepad.dpad_down;
        int slidePos = robot.getSlidePos();
        movingSlide = goingToGround || goingUp || goingDown;
        if (movingSlide)
            lastSlidePos = slidePos;

        int setPos = slidePos;
        if (goingToGround) {
            setPos = Inktonaut.SLIDES_BOTTOM;
        } else if (goingUp) {
            setPos += 35;
        } else if (goingDown) {
            setPos -= 32;
        } else {
            movingSlide = false;
            setPos = lastSlidePos;
        }
        robot.setSlidePos(setPos);

//
//        // go up using setVelocity
//        if (goingUp) {
//            double velocity = Inktonaut.SLIDES_DEFAULT_SPEED;
//            if (slidePos > Inktonaut.SLIDES_TOP - Inktonaut.SLIDES_BUFFER)
//                velocity = 0;
//
//            robot.setSlideVelocity(velocity);
//        }
//        // go down using setVelocity
//        else if (goingDown) {
//            double velocity = Inktonaut.SLIDES_DEFAULT_SPEED * 0.8;
//            if (slidePos < Inktonaut.SLIDES_BOTTOM + Inktonaut.SLIDES_BUFFER)
//                velocity = 0;
//
//            robot.setSlideVelocity(0);
//        }
//        // use setPosition
//        else  {
//            int setPos = slidePos;
//            // move to ground
//            if (goingToGround) {
//                setPos = Inktonaut.SLIDES_BOTTOM;
//            }
//            // maintain position
//            else {
//                setPos = slidePos;
//            }
//
//            setPos = Range.clip(setPos, Inktonaut.SLIDES_BOTTOM, Inktonaut.SLIDES_TOP);
//            robot.setSlidePos(setPos);
//        }
//
////        int slidePos = robot.getSlidePos();
//        double up = -gamepad.right_stick_y;
//        double down = gamepad.right_stick_y;
//
//        if(gamepad.dpad_up) fastSlides = true;
//        else if(gamepad.dpad_down) fastSlides = false;
//        int SLIDE_UP_VELO = (int)(1000 * (fastSlides ? 1:0.6));
//        int SLIDE_DOWN_VELO = (int)(-800 * (fastSlides ? 1:0.45));
//        // joystick: continuous
//        if (up>0 || down>0) {
//            if(up>0 && slidePos >= Inktonaut.SLIDES_TOP-30) robot.setSlidePos(Inktonaut.SLIDES_TOP); //buffer for the top
//            else if(down>0 && slidePos <= Inktonaut.SLIDES_BOTTOM+30) robot.setSlidePos(Inktonaut.SLIDES_BOTTOM); //buffer for bottom
//            else {
//                movingSlide = true;
//                double velo = 0;
//                if (up > 0) { // ticks/s
//                    velo = SLIDE_UP_VELO; slideDirection = 1; }
//                else {
//                    velo = SLIDE_DOWN_VELO; slideDirection = -1; }
//                robot.setSlideVelocity(velo);
//            }
//            // a,b,x,y, preset positions or hold current position
//        }
//        else {
//            if (movingSlide) {
//                movingSlide = false;
//                lastSlidePos = slidePos + 5*slideDirection; //essentially stops moving the slides when you let go of the vertical stick
//            }
//            int setPos = lastSlidePos;
//            slideDirection = 0;
//            movingSlide = true;
//            if (gamepad.y) {setPos = 300;}
//            else if (gamepad.b) {setPos = 700;}
//            else if (gamepad.a || movingToGround) {setPos = Inktonaut.SLIDES_BOTTOM; }
//            else {movingSlide = false; }
//            // stop moving to ground if reached it
//            if (slidePos < 20) movingToGround = false;
//            setPos = Range.clip(setPos, Inktonaut.SLIDES_BOTTOM, Inktonaut.SLIDES_TOP);
//            robot.setSlidePos(setPos);
//        }
        return slidePos;
    }

    public static final double ARM_SYSTEM_SCALE = 0.015;
    private double interp = 0.0; // interp [0, 1]
    public double moveArmSystem(@NonNull Gamepad gamepad){ //returns interpolation value
        boolean up = gamepad.x;
        boolean down = gamepad.b;

        if(up) interp += ARM_SYSTEM_SCALE;

        else if(down) interp -= ARM_SYSTEM_SCALE;
        interp = Range.clip(interp, 0, 1);

        if(up || down){
            double wristPos = -Range.scale(interp, 0, 1, -Inktonaut.WRIST_POS_PIXEL, -Inktonaut.WRIST_POS_BACKDROP);
            double elbowPos = Range.scale(interp, 0, 1, Inktonaut.ELBOW_POS_PIXEL, Inktonaut.ELBOW_POS_BACKDROP);
            robot.setWristPos(wristPos);
            robot.setElbowPos(elbowPos);
        }

        return interp;
    }
    public static final double WRIST_POS_SCALE = 0.0085;
    public double moveWrist(@NonNull Gamepad gamepad){
        boolean goingUp = gamepad.right_trigger > 0;
        boolean goingDown = gamepad.left_trigger > 0;
        double wristPos = robot.getWristPos();
        if (goingUp) {
            wristPos += WRIST_POS_SCALE;
            robot.setWristPos(wristPos);
        } else if (goingDown) {
            wristPos -= WRIST_POS_SCALE;
            robot.setWristPos(wristPos);
        }

        return wristPos;
    }

    public static final double ELBOW_POS_SCALE = 0.085;
    public double moveElbow(@NonNull Gamepad gamepad){
        boolean goingUp = gamepad.right_bumper;
        boolean goingDown = gamepad.left_bumper;
        double elbowPos = robot.getElbowPos();
        if (goingUp) {
            elbowPos += ELBOW_POS_SCALE;
            robot.setElbowPos(elbowPos);
        } else if (goingDown) {
            elbowPos -= ELBOW_POS_SCALE;
            robot.setElbowPos(elbowPos);
        }

        return elbowPos;
    }
    public double moveClaw(@NonNull Gamepad gamepad) {
        boolean close = gamepad.right_bumper;
        boolean open = gamepad.left_bumper;

        if (close) {
            robot.closeClaw();
        }
        else if (open) {
            robot.openClaw();
        }
        return robot.getClawPos();
    }

    public void runOpMode() {
        robot = new Inktonaut(this);
        float speedFactor = 0.7f;

        robot.setClawPos(Inktonaut.CLAW_POS_CLOSE);
        robot.setWristPos(Inktonaut.WRIST_POS_PIXEL);
        robot.setElbowPos(Inktonaut.ELBOW_POS_PIXEL);
        robot.setSlidePos(Inktonaut.SLIDES_BOTTOM);

        waitForStart();
        while(opModeIsActive())
        {
            float[] driveTrainPowers = moveDriveTrain(gamepad1, speedFactor);
            int slidePos = moveSlides(gamepad2);
            double clawPos = moveClaw(gamepad1);
            double wristPos = moveWrist(gamepad2);
            double armSystem = moveArmSystem(gamepad2);
            double elbowPos = moveElbow(gamepad2);

//            telemetry.addData("imu", robot.imu.getAngularOrientation());
            telemetry.addData("Speed factor", speedFactor);
            telemetry.addData("Drivetrain powers", Arrays.toString(driveTrainPowers));
            telemetry.addData("Slide position", slidePos);
            telemetry.addData("Claw position", clawPos);
            telemetry.addData("Wrist position", wristPos);
            telemetry.addData("Arm System" , armSystem);
            telemetry.addData("Elbow position", elbowPos);
            telemetry.update();
        }
    }
}
