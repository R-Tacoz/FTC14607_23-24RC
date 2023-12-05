package org.firstinspires.ftc.teamcode.util.odometry;

import com.arcrobotics.ftclib.command.OdometrySubsystem;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.kinematics.HolonomicOdometry;

public class FTCLibOdometry {

    //HardWare
    private MotorEx encoderLeft, encoderRight, encoderPerp;
    //Math Variables
    private double TRACKWIDTH, TICKS_TO_INCHES, CENTER_WHEEL_OFFSET;

    OdometrySubsystem odometry;

    public FTCLibOdometry(MotorEx encoderLeft, MotorEx encoderRight, MotorEx encoderPerp,
                          double TRACKWIDTH, double CENTER_WHEEL_OFFSET, double TICKS_TO_INCHES
                          ){
        this.encoderLeft = encoderLeft;
        this.encoderRight = encoderRight;
        this.encoderPerp = encoderPerp;
        this.TRACKWIDTH = TRACKWIDTH;
        this.TICKS_TO_INCHES = TICKS_TO_INCHES;
        this.CENTER_WHEEL_OFFSET = CENTER_WHEEL_OFFSET;
    }

    public void init(){
        encoderLeft.setDistancePerPulse(TICKS_TO_INCHES);
        encoderRight.setDistancePerPulse(TICKS_TO_INCHES);
        encoderPerp.setDistancePerPulse(TICKS_TO_INCHES);

        HolonomicOdometry holOdom = new HolonomicOdometry(
                encoderLeft::getDistance,
                encoderRight::getDistance,
                encoderPerp::getDistance,
                TRACKWIDTH, CENTER_WHEEL_OFFSET
        );

        odometry = new OdometrySubsystem(holOdom);
    }

    public void update(){
        odometry.update();
    }
    public Pose2d getPose(){
        return odometry.getPose();
    }




}
