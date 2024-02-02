package org.firstinspires.ftc.teamcode.robots;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;

public class Outreach extends MecanumDrive {
    public DcMotorEx rightSlide, leftSlide;
    public DcMotorEx[] slides;
    public PIDFController slidepidfcontroller;

    public final static int SLIDEBOTTOM = 40;
    public final static int SLIDETOP = 960;

    public Outreach(LinearOpMode opmode){
        super(opmode);
        // Drivetrain Motors: goBilda 5203 Series Yellow Jacket Planetary Gear Motor, 312 RPM
        dimensions = new RobotDimensions(-1, -1, -1, 9.6, 145);

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

    public void resetSlideEncoders() {
        setRunMode(slides, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        // behavior may vary between motor models, so after resetting set all slides back to RUN_TO_POSITION
        for (DcMotorEx motor: slides) {
            motor.setTargetPosition(motor.getCurrentPosition());
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        }
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
