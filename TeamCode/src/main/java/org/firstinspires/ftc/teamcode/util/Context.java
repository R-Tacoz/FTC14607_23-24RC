package org.firstinspires.ftc.teamcode.util;

public class Context {
    public double x;
    public double y;
    public double theta; // heading; radians

    public Context() {
        x = 0f;
        y = 0f;
        theta = 0f;
    }

    public Context(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public String toString() {
        return "x: "+x+", y: "+y+", theta: "+theta;
    }

}
