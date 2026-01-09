package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class HardwareController {
    // Set constants
    public static final double INTAKE_POWER = 0.8;
    public static final double TRANSFER_POWER = 0.5;

    //public static final double FLYWHEEL_RPS = 2.3; // RPS
    public static final double FW_VEL_ERROR = 0.15; // RPS

    public static final double TURRET_ALIGNMENT_ERROR = 0.05; // Meters
    public static final double TURRET_REVOLUTION_TOLERANCE = Math.PI / 12; // Radians
    public static final int SEARCH_INCREMENT = 10; // Ticks

    // Turret constants
    public int turretPosition = 0;
    public int targetPosition = 0;
    public double targetSpeed = 33.0; // Target flywheel speed RPS
    public double hoodPosition = 0.5;
    public static final int TICKS_PER_REVOLUTION = 1470; // Ticks from motor per rotation
    public static final double TICKS_PER_DEGREE = 4.083;
    public static final int TURRET_TICK_LIMIT = 500;
    public static final double FLYWHEEL_TPR = 28.0; // TPS

    // Turret velocity PID variables
    public double turretVelocityError = 0.0;
    public double turretPreviousError = 0.0;
    public double turretTotalError = 0.0;
    public double targetVelocity = 0.0;
    public double Kp = 0.8;
    public double Kd = 0.0;
    public double Ki = 0.01;
    public double[] turretMovingError = new double[10];


    public static final double NORMAL_DRIVE_RPS = 5.0;
    public static final double PRECISE_DRIVE_RPS = 0.5;

    public static final double GOBILDA_TPR = 537.6; // TPS
    public static final double WHEEL_DIAMETER = 4.0; // Inches
    public static final double GEAR_RATIO = 10.0 / 15.0;
    public static final int TURRET_REVOLUTION_TOLERANCE_TICKS = (int) ((TURRET_REVOLUTION_TOLERANCE * GOBILDA_TPR) / (Math.PI * 2));

    // Declare variables

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;
    public Servo turretHood;
    private float drivetrainVelocityScale;

    public HardwareController(HardwareMap hardwareMap) {
        // Map drivetrain motors
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        // Map mechanism motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "turretFlywheel");
        turretYaw = hardwareMap.get(DcMotorEx.class, "turretYaw");

        // Map turret hood servo
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        // Reset flywheel yaw motor
        turretYaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        setAllToDefault();
    }

    private void setAllToDefault() {
        // Set drivetrain motor directions
        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);

        // Set mechanism motor directions
        intake.setDirection(DcMotorEx.Direction.REVERSE);
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        turretFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        turretYaw.setDirection(DcMotorEx.Direction.REVERSE);

        // Set servo direction
        turretHood.setDirection(Servo.Direction.FORWARD);

        // Set turret yaw motor to use encoder
        turretYaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretYaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretYaw.setTargetPosition(0);
        turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretYaw.setPower(0.5);

        turretFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretFlywheel.setPower(0.0);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        leftFront.setMode(mode);
        leftBack.setMode(mode);
        rightFront.setMode(mode);
        rightBack.setMode(mode);
    }

    public void setAllMode(DcMotor.RunMode mode) {
        // Set mode to drivetrain
        setDrivetrainMode(mode);
        // Set mode to all mechanisms
        intake.setMode(mode);
        transfer.setMode(mode);
        turretFlywheel.setMode(mode);
        turretYaw.setMode(mode);
    }

    /**
     * Toggles the flywheel between no power and the target velocity
     *
     * @param isOn if the flywheel should be on
    */

    public void toggleFlywheel(boolean isOn) {
        turretFlywheel.setVelocity(
                isOn ? targetSpeed * FLYWHEEL_TPR : 0.0
        );
    }

    /**
     * Gets the current flywheel velocity
     *
     * @return flywheel velocity in rps
    */
    public double getFlywheelVelocity() {
        return toRPS(
                turretFlywheel.getVelocity()
        );
    }
    public boolean conditionalFeed() {
        boolean flywheelInRange = getFlywheelVelocity() < targetSpeed + FW_VEL_ERROR && getFlywheelVelocity() > targetSpeed - FW_VEL_ERROR;
        // Feed only if flywheel is in acceptable velocity range
        if (flywheelInRange) {
            transfer.setPower(TRANSFER_POWER);
        } else {
            transfer.setPower(TRANSFER_POWER);
        }
        // Return result for external use
        return true;
    }

    /**
     * Converts a velocity in rotations-per-second to ticks-per-second
     *
     * @param rps rotations per second
     * @return ticks per second
    */
    public static double toTPS(double rps) { return rps * FLYWHEEL_TPR; }

    /**
     * Converts a velocity in ticks-per-second to rotations-per-second
     *
     * @param tps ticks per second
     * @return rotations per second
     */
    public static double toRPS(double tps) { return tps / FLYWHEEL_TPR; }

    /**
     * Unless it is already aligned, continuously sends the proper velocities to align turret
     * Assumes that the goal is in frame
     *
     * @param deltaAngle angle error in degrees between turret and april tag
    */
    public void updateTurretTarget(double deltaAngle) {
        int deltaTicks = (int) (deltaAngle * TICKS_PER_DEGREE);
        int currentPosition = turretYaw.getCurrentPosition();
        int tempPos = currentPosition + deltaTicks;
        targetPosition = Math.max(Math.min(tempPos, TURRET_TICK_LIMIT), -TURRET_TICK_LIMIT);
        turretYaw.setTargetPosition(targetPosition);

    }

    /**
     * Resets turret position, hood angle, and flywheel speed to default
     * values
     */
    public void resetTurret() {
        turretYaw.setTargetPosition(0);
        targetSpeed = 33.0;
        turretHood.setPosition(0.5);
    }

    /**
     * Updates flywheel velocity and hood angle based on regression
     * using calculated distance between turret and april tag
     *
     * @param distance distance in cm between limelight and april tag
     */
    public void updateFlywheel(double distance) {
        targetSpeed = 0.11 * distance + 33.5;
        hoodPosition = Math.max(Math.min(0.52 - (0.0016 * distance) + (0.00000301 * Math.pow(distance, 2)), 0.5), 0.3);
        turretHood.setPosition(hoodPosition);
        setFlywheelVelocity(targetSpeed);
    }

    /**
     * PID controller for turret velocity
     *
     * @param deltaTime time since last run cycle
     * @return power output power for flywheel motor
     */
    public double PID_Controller(double deltaTime) {
        turretVelocityError = targetVelocity - (turretFlywheel.getVelocity() / FLYWHEEL_TPR); // P-value
        double turretDeltaError = (turretVelocityError - turretPreviousError) / deltaTime; // D-value
        //turretTotalError += (turretVelocityError * deltaTime); // I-value

        // Calculate moving integral error
        System.arraycopy(turretMovingError, 0, turretMovingError, 1, turretMovingError.length - 1);
        turretMovingError[0] = turretVelocityError;
        turretTotalError = 0.0;
        for (double error : turretMovingError) {turretTotalError += error;}

        turretPreviousError = turretVelocityError;

        double power = (Kp * turretVelocityError) + (Kd * turretDeltaError) + (Ki * turretTotalError);
        power = Math.max(Math.min(power, 1.0), -1.0); // Limit power from -1.0 - 1.0
        turretFlywheel.setPower(power);
        return power;
    }

    /**
     * Set flywheel velocity and reset PID controller
     *
     * @param velocity target velocity for flywheel in ticks/s
     */
    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
        //turretTotalError = 0.0;
        turretPreviousError = 0.0;
    }
}
