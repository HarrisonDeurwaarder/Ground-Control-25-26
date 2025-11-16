package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorDriver {
    public static final double INTAKE_POWER = 0.6;
    public static final double TRANSPORT_POWER = 0.6;
    public static final double FLYWHEEL_SPEED = 5.0; // Revolutions per second

    public static final double TRIGGER_THRESHOLD = 0.05;
    public static final double MAX_DRIVE_POWER = 1.0;
    public static final double PRECISE_DRIVE_POWER = 0.15;
    public static final double TPR = 537.6;

    public DcMotor frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, intakeMotor, transportMotor;
    public DcMotorEx flywheelMotor;

    public MotorDriver(HardwareMap hardwareMap) {
        /*
         * Get motor configurations from hardware map
        */

        frontLeftDrive = hardwareMap.get(DcMotor.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotor.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotor.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotor.class, "rightBack");

        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        transportMotor = hardwareMap.get(DcMotor.class, "transport");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        /*
         * Send default power
         * This is idle for all motors
        */

        frontLeftDrive.setPower(0.0);
        frontRightDrive.setPower(0.0);
        backLeftDrive.setPower(0.0);
        backRightDrive.setPower(0.0);

        intakeMotor.setPower(0.0);
        transportMotor.setPower(0.0);

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

        /*
         * Set the flywheel motor to run via encoder for speed tracking
        */

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelMotor.setVelocity(0.0);
    }

    public void setDrivetrainPower(
        double frontLeftPower,
        double backLeftPower,
        double frontRightPower,
        double backRightPower,
        double maxPower
    ) {
        frontLeftDrive.setPower(frontLeftPower * maxPower);
        frontRightDrive.setPower(frontRightPower * maxPower);
        backLeftDrive.setPower(backLeftPower * maxPower);
        backRightDrive.setPower(backRightPower * maxPower);
    }
}
