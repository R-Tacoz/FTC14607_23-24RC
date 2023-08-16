package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ThreeWheelOdometry implements Odometry {
    private Context context;
    private final double  odoWidth, odoBackDist;
    private DcMotor rightEncoder, leftEncoder, backEncoder;
    private final double TICKS_TO_DISTANCE;

    public ThreeWheelOdometry(
        DcMotor rightEncoder,
        DcMotor leftEncoder,
        DcMotor backEncoder,
        int odoTicks,
        float odoWheelDiam,
        float odoWidth,
        float odoBackDist
    ) {
        this.rightEncoder = rightEncoder;
        this.leftEncoder = leftEncoder;
        this.backEncoder = backEncoder;
        this.odoWidth = odoWidth;
        this.odoBackDist = odoBackDist;
        this.TICKS_TO_DISTANCE = Math.PI * odoWheelDiam / odoTicks;
        this.context = new Context();
    }

    public Context getTickContext() { return this.context; }

    public Context getDistContext() { return Context.multiply(this.context, this.TICKS_TO_DISTANCE); }

    @Override
    public Context getContext() { return getDistContext(); }

    public void updateContext(int deltaRight, int deltaLeft, int deltaBack) {
        double cosTheta = Math.cos(context.theta);
        double sinTheta = Math.sin(context.theta);
        double forwardAvg = (deltaRight + deltaLeft) / 2.0;
        context.x += (forwardAvg * sinTheta) + (deltaBack * cosTheta);
        context.y += (forwardAvg * cosTheta) + (deltaBack * sinTheta);
        context.theta += Math.atan( (deltaRight - deltaLeft) / odoWidth );

//        // MacLaurin Approximations for cos, sin, tan (degree of 2) & no forward average
//        double cosTheta = 1 - Math.pow(context.theta, 2) / 2.0;
//        double sinTheta = context.theta;
//        double forwardAvg = deltaLeft;
//        context.x += (forwardAvg * sinTheta) + (deltaBack * cosTheta);
//        context.y += (forwardAvg * cosTheta) + (deltaBack * sinTheta);
//        context.theta += (deltaRight - deltaLeft) / odoWidth;
    }
}
