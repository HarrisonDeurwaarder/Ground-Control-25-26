package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorDriverPID {
    public static final double INTAKE_RPS = 2.0;
    public static final double TRANSPORT_RPS = 2.0;
    public static final double FLYWHEEL_RPS = 3.0;

    public static final double TRIGGER_THRESHOLD = 0.05;
    public static final double FW_VEL_ERROR = 2.0;

    public static final double MAX_DRIVE_RPS = 5.0;
    public static final double PRECISE_DRIVE_RPS = 0.5;

    public static final double TPR = 537.6;
    public static final double WHEEL_DIAMETER = 4.0; // Inches
    public static final double GEAR_RATIO = 10.0 / 15.0;

    public DcMotorEx frontLeftDrive, backLeftDrive, frontRightDrive, backRightDrive, intakeMotor, transportMotor, flywheelMotor;

    public MotorDriverPID(HardwareMap hardwareMap) {
        /*
         * Get motor configurations from hardware map
        */

        frontLeftDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        backLeftDrive = hardwareMap.get(DcMotorEx.class, "leftBack");
        frontRightDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        backRightDrive = hardwareMap.get(DcMotorEx.class, "rightBack");

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
         * Motors on the left side, flywheel, and intake need to be reversed
         */

        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        transportMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.FORWARD);

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

    public void setDrivetrainPower(double velocity) {
        frontLeftDrive.setVelocity(velocity);
        frontRightDrive.setVelocity(velocity);
        backLeftDrive.setVelocity(velocity);
        backRightDrive.setVelocity(velocity);
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

    public boolean flywheelInRange() {
        // Convert the TPS velocity of the flywheel to RPS
        double velocityRPS = MotorDriverPID.toRPS(flywheelMotor.getVelocity());
        // Verify that the (converted to RPS) velocity is within the lower/upper bound
        return ((velocityRPS >= FLYWHEEL_RPS - FW_VEL_ERROR) && (velocityRPS <= FLYWHEEL_RPS + FW_VEL_ERROR));
    }

    public static double toTPS(double rps) {
        return rps * TPR;
    }

    public static double toRPS(double tps) {
        return tps / TPR;
    }
}
