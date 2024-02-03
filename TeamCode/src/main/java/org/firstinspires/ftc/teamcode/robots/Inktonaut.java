package org.firstinspires.ftc.teamcode.robots;

import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

public class Inktonaut extends MecanumDrive {

    public DcMotorEx slides; // goBilda Yellow Jacket 1150 RPM
    public DcMotorEx suspensionRight, suspensionLeft; // goBilda Yellow Jacket 312 RPM;

    public Servo claw; // goBilda Dual Mode Torque
    public Servo wrist; // Axon mini
    public Servo elbow; // Axon mini
    public Servo gliderRelease; // naughty naughty arnav

    public PIDFController slidepidfcontroller;

    // Drivetrain Motors: goBilda Yellow Jacket 312 RPM
    public final static RobotDimensions DIMENSIONS = new RobotDimensions(
            -1, -1, -1, 9.6, 537.7
    );
    public final static int SLIDES_BOTTOM = 0; // ticks
    public final static int SLIDES_TOP = 670;
    public final static double SLIDES_DEFAULT_SPEED = 300; // ticks / sec
    public final static double SLIDES_BUFFER = 30; // ticks
    public final static double WRIST_POS_PIXEL = 0.66;
    public final static double WRIST_POS_BACKDROP = 0.43;
    public final static double ELBOW_POS_PIXEL = 0.2;
    public final static double ELBOW_POS_BACKDROP = 0.91;
    public final static double CLAW_POS_OPEN = 0.325;
    public final static double CLAW_POS_CLOSE = 0.6;

    public Inktonaut(LinearOpMode opModeInstance) {
        super(opModeInstance);
        dimensions = Inktonaut.DIMENSIONS;

        slides = hardwareMap.get(DcMotorEx.class, "slides");
        claw = hardwareMap.get(Servo.class, "claw");
        wrist = hardwareMap.get(Servo.class, "wrist");
        elbow = hardwareMap.get(Servo.class, "elbow");
      //  gliderRelease = hardwareMap.get(Servo.class, "gliderRelease");

        slides.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slides.setDirection(DcMotor.Direction.REVERSE);
        // TODO: make variables if this matters
//        slides.setVelocityPIDFCoefficients(15.0, 2.0, 0.0, 0);
//        slides.setPositionPIDFCoefficients(10.0);

        resetSlideEncoders();

        slidepidfcontroller = new PIDFController(50, 0.05, 0, 0);
        //slidepidfcontroller.setIntegrationBounds(-5, 5);
        slidepidfcontroller.setTolerance(3);
    }

    // ------------------------------------- MISC METHODS ------------------------------------------

    /**
     * Assigns the current position of both slide motors to tick (position) 0.
     * MAKE SURE THE SLIDES ARE AT THE BOTTOM WHEN DOING THIS.
     */
    public void resetSlideEncoders() {
        slides.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // behavior may vary between motor models, so after resetting set all slides back to RUN_TO_POSITION
        slides.setTargetPosition(slides.getCurrentPosition());
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    // --------------------------------- ARM METHODS ------------------------------------------

    public double getClawPos() {
        return claw.getPosition();
    }
    public void setClawPos(double pos)  {
        claw.setPosition(Range.clip(pos, CLAW_POS_OPEN, CLAW_POS_CLOSE));
    }
    public void closeClaw() { setClawPos(CLAW_POS_CLOSE); }
    public void openClaw() { setClawPos(CLAW_POS_OPEN); }
    public double getWristPos() {
        return wrist.getPosition();
    }
    public void setWristPos(double pos) {
        wrist.setPosition(Range.clip(pos, WRIST_POS_BACKDROP, WRIST_POS_PIXEL));
    }
    public double getElbowPos() {
        return elbow.getPosition();
    }
    public void setElbowPos(double pos) {
        elbow.setPosition(Range.clip(pos, ELBOW_POS_PIXEL, ELBOW_POS_BACKDROP));
    }

    /**
     * Move the elbow while mainting the angle of the claw (for outtaking on backdrop)
     * @param pos - [0.0, 1.0]
     */
    public void setElbowIK(double pos) { //Inverse kinematics

    }

    // ------------------------------------ INTERACTOR METHODS -------------------------------------

    /** @return Left slides current position in ticks~(0-960) */
    public int getSlidePos() { return slides.getCurrentPosition(); }

    /** @param velocity - (ticks / sec) */
    public void setSlideVelocity(double velocity){
        double currentPos = getSlidePos();
        // prevents setting velocity if it will exceed the limits
        if ( (currentPos>=SLIDES_TOP && velocity>0) || (currentPos<=SLIDES_BOTTOM && velocity<0) )
            velocity = 0;
        slides.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        slides.setVelocity(velocity);
    }

    /** @param height - (ticks)     */
    public void setSlidePos(int height) {
        height = Range.clip(height, SLIDES_BOTTOM, SLIDES_TOP);
        slides.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slides.setTargetPosition(height);
        slides.setVelocity( (height > getSlidePos()) ? 1200:1000 ); //going up is faster than going down
    }

    public void setSuspensionPos(double pos) {

    }

}
