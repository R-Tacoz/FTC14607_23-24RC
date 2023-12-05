package org.firstinspires.ftc.teamcode.util.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

public class MyOdometry {
    private Pose pose;
    private final double  odoWidth, odoBackDist;
    private DcMotor rightEncoder, leftEncoder, backEncoder;
    private final double TICKS_TO_DISTANCE;

    public MyOdometry(
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
        this.pose = new Pose();
    }

    public Pose getTickContext() { return this.pose; }

    public Pose getDistContext() { return Pose.multiply(this.pose, this.TICKS_TO_DISTANCE); }

    public Pose getPose() { return getDistContext(); }

    public void updateContext(int deltaRight, int deltaLeft, int deltaBack) {
        double cosTheta = Math.cos(pose.theta);
        double sinTheta = Math.sin(pose.theta);
        double forwardAvg = (deltaRight + deltaLeft) / 2.0;
        pose.x += (forwardAvg * sinTheta) + (deltaBack * cosTheta);
        pose.y += (forwardAvg * cosTheta) + (deltaBack * sinTheta);
        pose.theta += Math.atan( (deltaRight - deltaLeft) / odoWidth );

//        // MacLaurin Approximations for cos, sin, tan (degree of 2) & no forward average
//        double cosTheta = 1 - Math.pow(context.theta, 2) / 2.0;
//        double sinTheta = context.theta;
//        double forwardAvg = deltaLeft;
//        context.x += (forwardAvg * sinTheta) + (deltaBack * cosTheta);
//        context.y += (forwardAvg * cosTheta) + (deltaBack * sinTheta);
//        context.theta += (deltaRight - deltaLeft) / odoWidth;
    }
}
