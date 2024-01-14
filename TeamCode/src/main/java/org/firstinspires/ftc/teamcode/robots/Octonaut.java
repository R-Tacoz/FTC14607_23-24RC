package org.firstinspires.ftc.teamcode.robots;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

public class Octonaut extends MecanumDrive {
    public int odoTicks;
    public CRServo clawServo;
    public Servo lift;

    public DcMotorEx rightSlide, leftSlide;
    public DcMotorEx[] slides;
    // controllers
    public PIDFController slidepidfcontroller;

    public final static RobotDimensions DIMENSIONS = new RobotDimensions(-1, -1, -1, 9.6, 537.7);
    public final static int SLIDEBOTTOM = 40;
    public final static int SLIDETOP = 960;
    public final static int GROUND = 40;

    public Octonaut(LinearOpMode opModeInstance) {
        super(opModeInstance);
        // Drivetrain Motors: goBilda 5203 Series Yellow Jacket Planetary Gear Motor, 312 RPM
        dimensions = Octonaut.DIMENSIONS;

//        odometry = new ThreeWheelOdometry(
//            hardwareMap.get(DcMotor.class, "odoRight"),
//            hardwareMap.get(DcMotor.class, "odoLeft"),
//            hardwareMap.get(DcMotor.class, "odoBack"),
//            odoTicks,
//            odoWheelDiam,
//            odoWidth,
//            odoBackDist
//        );

        // claw
        clawServo = hardwareMap.get(CRServo.class, "claw");
        lift = hardwareMap.get(Servo.class, "lift");

        //slides
        rightSlide = hardwareMap.get(DcMotorEx.class, "RightSlide");
        leftSlide = hardwareMap.get(DcMotorEx.class, "LeftSlide");
        slides = new DcMotorEx[]{rightSlide, leftSlide};
        for(DcMotorEx m: slides) {
            m.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            m.setVelocityPIDFCoefficients(15.0, 2.0, 0.0, 0);
            m.setPositionPIDFCoefficients(10.0);
        }
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
        setRunMode(slides, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // behavior may vary between motor models, so after resetting set all slides back to RUN_TO_POSITION
        for (DcMotorEx motor: slides) {
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
    }

    // --------------------------------- CLAW METHODS ------------------------------------------
    public void outtake(){
        clawServo.setDirection(CRServo.Direction.FORWARD);
    }

    public void intake(){
        clawServo.setDirection(CRServo.Direction.REVERSE);
    }

    public void setClawPower(double power){
        clawServo.setPower(power);
    }

    public void setLift(double pos) {
        double position = Range.clip(pos, 0, 1);
        lift.setPosition(position);
    }

    // ------------------------------------ INTERACTOR METHODS -------------------------------------

    /**
     * Returns the slides current position
     * @return Left slides current position ~(0-960)
     */
    public int getSlidePos() { return leftSlide.getCurrentPosition(); }

    /**
     * Sets the velocity of the slides
     * @param velocity
     */
    public void setSlideVelo(double velocity){
        double currentPos = getSlidePos();
        // prevents setting velocity if it will exceed the limits
        if ( (currentPos>=955 && velocity>0) || (currentPos<=0 && velocity<0) ) velocity = 0;
        setRunMode(slides, DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightSlide.setVelocity(-velocity);
        leftSlide.setVelocity(velocity);
    }

    /**
     * Extend/retract slides to desired height using encoders
     * @param height - (ticks)
     */
    public void setSlidePos(int height) {
        height = Range.clip(height, SLIDEBOTTOM, SLIDETOP);
        int change = height - getSlidePos();
        rightSlide.setTargetPosition(-height);
        leftSlide.setTargetPosition(height);
        for(DcMotorEx slide:slides) {
            slide.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            slide.setVelocity( (change>0)? 1200:1000 );
        }
    }
}
