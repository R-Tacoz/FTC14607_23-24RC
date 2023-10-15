package org.firstinspires.ftc.teamcode.Robots;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

/**
 * RobotBase is to be extended by all hardware control classes for 4-wheel Mechanum drivetrain
 * functionality for Autonomous and Driver-Controlled purposes
 */
@Config
public class RobotBase {
    // op mode
    public LinearOpMode opMode;
    public Telemetry telemetry;
    public HardwareMap hardwareMap;
    // drivetrain
    public DcMotorEx frontRight, frontLeft, backRight, backLeft;
    public DcMotorEx[] drivetrain;
    // sensor/controllers
    public BNO055IMU imu;

    public static PIDFCoefficients internalDrivePosPIDF = new PIDFCoefficients(10, 0.05, 0, 0, MotorControlAlgorithm.LegacyPID);
    public static PIDFCoefficients internalDriveVeloPIDF = new PIDFCoefficients(10, 3, 0, 0, MotorControlAlgorithm.LegacyPID);

    public static double rotateP = 3.5, rotateI = 0.4, rotateD = 0.2, rotateF = 0.3;
    public static PIDFController rotatePIDF;

    public PIDFController[] strafePIDFs; // for enhanced strafe
    public static PIDFController
            FRStrafePID = new PIDFController(1.47,0.06,0.1,0.3),
            FLStrafePID = new PIDFController(1.6,0.06,0.1,0.3),
            BRStrafePID = new PIDFController(1.7,0.06,0.1,0.3),
            BLStrafePID = new PIDFController(1.9,0.06,0.1,0.3)
    ;

    public PIDFController[] drivePIDFs; // for enhanced forward
    public static double
            frP=2.8, frI=0, frD=1.21, frF=0.2,
            flP=2.8, flI=0, flD=1.21, flF=0.21,
            brP=2.8, brI=0, brD=1.21, brF=0.2,
            blP=2.8, blI=0, blD=1.21, blF=0.21,
            headKeepP=40, headKeepI=0, headKeepD=10, headKeepF=0;
    public static PIDFController FRPIDF, FLPIDF, BRPIDF, BLPIDF, headKeepPIDF;
    // hardware properties
    public final int DRIVETRAIN_TICKS; //
    public final double WHEEL_DIAMETER, WHEEL_CIRCUM; // cm

    /**
     * Instantiate all variables, initialize imu & PID
     * @param opModeInstance Pass using "this" keyword
     * @param drivetrainTicks
     * @param wheelDiameter
     */
    public RobotBase(LinearOpMode opModeInstance, int drivetrainTicks, double wheelDiameter) {
        opMode = opModeInstance;
        telemetry = opMode.telemetry;
        hardwareMap = opMode.hardwareMap;

        // set bulk reads to auto
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : hubs)
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);

        //drivetrain
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);
        drivetrain = new DcMotorEx[]{frontRight, frontLeft, backRight, backLeft};
        for (DcMotorEx motor : drivetrain) {
//            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, internalDrivePosPIDF);
//            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, internalDriveVeloPIDF);
            motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        }

        strafePIDFs = new PIDFController[]{FRStrafePID, FLStrafePID, BRStrafePID, BLStrafePID};
        for(PIDFController pid : strafePIDFs) pid.setTolerance(1);

        // enhanced forward
        FRPIDF = new PIDFController(frP, frI, frD, frF);
        FLPIDF = new PIDFController(flP, flI, flD, flF);
        BRPIDF = new PIDFController(brP, brI, brD, brF);
        BLPIDF = new PIDFController(blP, blI, blD, blF);
        drivePIDFs = new PIDFController[]{FRPIDF, FLPIDF, BRPIDF, BLPIDF};
        for(PIDFController pid : drivePIDFs) pid.setTolerance(1);
        headKeepPIDF = new PIDFController(headKeepP, headKeepI, headKeepD, headKeepF);
        headKeepPIDF.setTolerance(0.1);

        rotatePIDF = new PIDFController(rotateP, rotateI, rotateD, rotateF);
        rotatePIDF.setTolerance(0.8);
//        // odo
//        odoRight = hardwareMap.get(DcMotorEx.class, "odoRight");
//        odoLeft = hardwareMap.get(DcMotorEx.class, "odoLeft");
//        odoBack = hardwareMap.get(DcMotorEx.class, "odoBack");

        //imu
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters imuparams = new BNO055IMU.Parameters();
        imuparams.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imuparams.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        imuparams.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        imuparams.loggingEnabled = true;
        imuparams.loggingTag = "IMU";
        imu.initialize(imuparams);

        // init constants
        this.DRIVETRAIN_TICKS = drivetrainTicks;
        this.WHEEL_DIAMETER = wheelDiameter;
        this.WHEEL_CIRCUM = Math.PI * wheelDiameter;
    }

    // ------------------------------------- MISC METHODS ------------------------------------------
    /**
     * Assigns the current position of every motor on the drivetrain to tick (position) 0
     */
    public void resetDriveTrainEncoders() { setRunMode(drivetrain, DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);}

    /**
     * Sets all the motors in motors[] to the passed run mode
     * @param motors drivetrain or slides
     * @param mode DcMotorEx.RunMode
     */
    public void setRunMode(DcMotorEx[] motors, DcMotorEx.RunMode mode) {
        for (DcMotorEx motor : motors) motor.setMode(mode);
    }

    /**
     * Converts a distance (IN CENTIMETERS) on the field to a number of ticks for the motor to rotate about
     * @param distance (centimeters)
     * @return Ticks for the drivetrain motors to turn
     */
    public int distanceToTicks(double distance, boolean strafe) {
        return (int) Math.round( (distance/this.WHEEL_CIRCUM * this.DRIVETRAIN_TICKS) * (strafe ? 1.2:1)  );
    }

    public double fixAngle(double angle) {
        return 0;
    }

    /**
     * Pauses op mode and interactors while drivetrain motors are running using RUN_TO_POSITION
     * @param startTime Pass System.nanoTime() or opmode.time here, or -1 if you dont want to break after 10s
     */
    public void blockExecutionForRunToPosition(long startTime) {
        while (opMode.opModeIsActive() && !opMode.isStopRequested()) {
            if (!drivetrain[0].isBusy()) break;
                // stop motors after 10 seconds
            else if (startTime > 0 && (System.nanoTime() - startTime > 10000000000L)) {
                for(DcMotorEx m : drivetrain) m.setVelocity(0);
                break;
            }
        }
    }

    // --------------------------------- MOVEMENT METHODS ------------------------------------------

    public void brake() {
        for (DcMotorEx motor : drivetrain) motor.setVelocity(0);
    }

    // all basic movement methods use distance in cm and speed in ticks/sec
    public void forward(double distance, double speed) {
        resetDriveTrainEncoders();
        int calculatedTicks = distanceToTicks(distance, false);
        frontRight.setTargetPosition(calculatedTicks);
        frontLeft.setTargetPosition(calculatedTicks);
        backRight.setTargetPosition(calculatedTicks);
        backLeft.setTargetPosition(calculatedTicks);
        for (DcMotorEx motor : drivetrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setVelocity(speed);
        }
        blockExecutionForRunToPosition(-1);
    }

    public void backward(double distance, double speed) {
        resetDriveTrainEncoders();
        int calculatedTicks = distanceToTicks(distance, false);
        frontRight.setTargetPosition(-calculatedTicks);
        frontLeft.setTargetPosition(-calculatedTicks);
        backRight.setTargetPosition(-calculatedTicks);
        backLeft.setTargetPosition(-calculatedTicks);
        for (DcMotorEx motor : drivetrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setVelocity(speed);
        }
        blockExecutionForRunToPosition(-1);
    }

    public void right(double distance, double speed) {
        resetDriveTrainEncoders();
        int calculatedTicks = distanceToTicks(distance, false);
        frontRight.setTargetPosition(-calculatedTicks);
        frontLeft.setTargetPosition(calculatedTicks);
        backRight.setTargetPosition(calculatedTicks);
        backLeft.setTargetPosition(-calculatedTicks);
        for (DcMotorEx motor : drivetrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setVelocity(speed);
        }
        blockExecutionForRunToPosition(System.nanoTime());
    }

    public void left(double distance, double speed) {
        resetDriveTrainEncoders();
        int calculatedTicks = distanceToTicks(distance, false);
        frontRight.setTargetPosition(calculatedTicks);
        frontLeft.setTargetPosition(-calculatedTicks);
        backRight.setTargetPosition(-calculatedTicks);
        backLeft.setTargetPosition(calculatedTicks);
        for (DcMotorEx motor : drivetrain) {
            motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            motor.setVelocity(speed);
        }
        blockExecutionForRunToPosition(System.nanoTime());
    }

    /**
     * forward but uses separate external pid for each motor and has angle correction
     * @param distance
     */
    public void forwardExp(double distance) {
        resetDriveTrainEncoders();
        int tickDistance = distanceToTicks(distance, false);
        for(PIDFController pid : drivePIDFs) {
            pid.reset();
            pid.setSetPoint(tickDistance);
        }
        double firstHeading = imu.getAngularOrientation().firstAngle;
        boolean flip = firstHeading < -90 || firstHeading > 90;
        headKeepPIDF.setSetPoint(flip && firstHeading<0 ? -firstHeading:firstHeading);
        do {
            //float angleCorrection = 0;
            float heading = imu.getAngularOrientation().firstAngle;
            // account for the fact that the angle goes 179, 180, then -180, -179
            double angleCorrection = headKeepPIDF.calculate(heading + (flip && heading<0 ? 360:0));
            frontRight.setVelocity(FRPIDF.calculate(frontRight.getCurrentPosition()) + angleCorrection);
            frontLeft.setVelocity(FLPIDF.calculate(frontLeft.getCurrentPosition()) - angleCorrection);
            backRight.setVelocity(BRPIDF.calculate(backRight.getCurrentPosition()) + angleCorrection);
            backLeft.setVelocity(BLPIDF.calculate(backLeft.getCurrentPosition()) - angleCorrection);
        } while(opMode.opModeIsActive() && !opMode.isStopRequested() &&
                !drivePIDFs[0].atSetPoint() && !drivePIDFs[1].atSetPoint() &&
                !drivePIDFs[2].atSetPoint() && !drivePIDFs[3].atSetPoint());

        brake();
    }

    /**
     * Experimental strafe that uses imu to maintain orientation
     * @param distance (cm) positive is right, negative is left
     */
    public void strafeExp(double distance) {
        resetDriveTrainEncoders();
        int tickDistance = distanceToTicks(distance, false);
        for(PIDFController pid : strafePIDFs) pid.reset();
        FRStrafePID.setSetPoint(-tickDistance); //fr
        FLStrafePID.setSetPoint(tickDistance);
        BRStrafePID.setSetPoint(tickDistance); //br
        BLStrafePID.setSetPoint(-tickDistance);
        double firstHeading = imu.getAngularOrientation().firstAngle;
        boolean flip = firstHeading < -90 || firstHeading > 90;
        headKeepPIDF.setSetPoint(flip && firstHeading<0 ? -firstHeading:firstHeading);
        do {
            float angleCorrection = 0;
            float heading = imu.getAngularOrientation().firstAngle;
            // account for the fact that the angle goes 179, 180, then -180, -179
            //float angleCorrection = headingPIDFController.calculate(heading + (flip && heading<0 ? 360:0));
            frontRight.setVelocity(FRStrafePID.calculate(frontRight.getCurrentPosition()) + angleCorrection);
            frontLeft.setVelocity(FLStrafePID.calculate(frontLeft.getCurrentPosition()) - angleCorrection);
            backRight.setVelocity(BRStrafePID.calculate(backRight.getCurrentPosition()) + angleCorrection);
            backLeft.setVelocity(BLStrafePID.calculate(backLeft.getCurrentPosition()) - angleCorrection);
        } while(!strafePIDFs[0].atSetPoint() && !strafePIDFs[1].atSetPoint() &&
                !strafePIDFs[2].atSetPoint() && !strafePIDFs[3].atSetPoint());

        for(DcMotorEx m : drivetrain) m.setVelocity(0);

    }

    /**
     * Rotates the robot [degrees] clockwise using PIDFController with imu as input. Should not be used
     * in driver-controlled programs
     * @param degrees angle to rotate to in degrees, SHOULD BE BETWEEN -180 AND 180
     */
    public void rotate(double degrees) {
        double motorSpeed = 0;
        float lastAngle = -imu.getAngularOrientation().firstAngle;
        degrees += lastAngle;
        boolean flip = degrees > 179 || degrees < -179;
        boolean sign = lastAngle >= 0; // true if angle positive, false if negative
        rotatePIDF.reset();
        rotatePIDF.setSetPoint(degrees);
        do {
            lastAngle = -imu.getAngularOrientation().firstAngle;
            if (flip) {
                // if angle was originally positive but the current angle is negative
                // add 360 to flip the sign
                if (sign && lastAngle < 0) lastAngle += 360;
                else if (!sign && lastAngle > 0) lastAngle -= 360;
            }
            motorSpeed = rotatePIDF.calculate(lastAngle);
            telemetry.addData("Motor Speed", motorSpeed);
            telemetry.addData("degrees", degrees);
            telemetry.addData("current angle", lastAngle);
            telemetry.addData("flip", flip);
            telemetry.addData("sign", sign);
            telemetry.update();
            frontRight.setVelocity(-motorSpeed);
            frontLeft.setVelocity(motorSpeed);
            backRight.setVelocity(-motorSpeed);
            backLeft.setVelocity(motorSpeed);

        } while (!rotatePIDF.atSetPoint());
        //make sure motors stop
        for(DcMotorEx m : drivetrain) m.setVelocity(0);
        rotatePIDF.reset();
    }


}