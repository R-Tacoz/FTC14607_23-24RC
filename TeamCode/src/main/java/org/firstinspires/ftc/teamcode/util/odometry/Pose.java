package org.firstinspires.ftc.teamcode.util.odometry;

public class Pose {
    public double x;
    public double y;
    public double theta; // heading; radians

    public Pose() {
        x = 0f;
        y = 0f;
        theta = 0f;
    }

    public Pose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public String toString() {
        return "x: "+x+", y: "+y+", theta: "+theta;
    }

    public static Pose multiply(Pose toConvert, double factor) {
        return new Pose(toConvert.x * factor, toConvert.y * factor, toConvert.theta);
    }

}
