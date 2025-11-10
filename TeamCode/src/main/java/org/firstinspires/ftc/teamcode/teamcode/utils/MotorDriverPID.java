package org.firstinspires.ftc.teamcode.teamcode.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorDriverPID {
    public static final double INTAKE_RPS = 2.0;
    public static final double TRANSPORT_RPS = 2.0;
    public static final double FLYWHEEL_RPS = 3.5; // Revolutions per second

    public static final double TRIGGER_THRESHOLD = 0.05;
    public static final double MAX_DRIVE_RPS = 5.0;
    public static final double PRECISE_DRIVE_RPS = 0.5;
    public static final double TPR = 537.6;

    public DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, intakeMotor, transportMotor, flywheelMotor;

    public MotorDriverPID(HardwareMap hardwareMap) {
        /*
         * Get motor configurations from hardware map
        */

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "frontleft");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "backleft");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "frontright");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "backright");

        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transportMotor = hardwareMap.get(DcMotorEx.class, "transport");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        /*
        * Set motors to run using encoder
        * Enables PID
        */

        frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        transportMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Send default power
         * This is idle for all motors
        */

        frontLeftDrive.setVelocity(0.0);
        frontRightDrive.setVelocity(0.0);
        backLeftDrive.setVelocity(0.0);
        backRightDrive.setVelocity(0.0);

        intakeMotor.setVelocity(0.0);
        transportMotor.setVelocity(0.0);
        flywheelMotor.setVelocity(0.0);

        /*
         * Set directions for each motor
         * Motors on the left side and flywheel need to be reversed
        */

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        transportMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        /*
         * Force motors to break under no power
         * Improves handling and increases drive reliability
        */

        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDrivetrainPower(
        double frontLeftPower,
        double backLeftPower,
        double frontRightPower,
        double backRightPower,
        double maxVelocity
    ) {
        frontLeftDrive.setVelocity(frontLeftPower * maxVelocity);
        frontRightDrive.setVelocity(frontRightPower * maxVelocity);
        backLeftDrive.setVelocity(backLeftPower * maxVelocity);
        backRightDrive.setVelocity(backRightPower * maxVelocity);
    }

    public static double toTPS(double rps) {
        return rps * TPR;
    }

    public static double toRPS(double tps) {
        return tps / TPR;
    }
}
