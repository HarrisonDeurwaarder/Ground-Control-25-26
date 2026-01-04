package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.acmerobotics.roadrunner.Vector2d;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

public class HardwareController {
    // Set constants
    public static final double INTAKE_POWER = 0.5;
    public static final double TRANSFER_POWER = 0.5;

    public static final double DEFAULT_FLYWHEEL_RPS = 2.3; // RPS
    public static final double FW_VEL_ERROR = 0.15; // RPS

    // Turret constants
    public int targetPosition = 0;
    public double targetSpeed = 33.0; // Target flywheel speed RPS
    public double hoodPosition = 0.5;

    // Turret velocity PID variables
    public double turretVelocityError = 0.0;
    public double turretPreviousError = 0.0;
    public double turretTotalError = 0.0;
    public double targetVelocity = 0.0;
    public double Kp = 0.0;
    public double Kd = 0.0;
    public double Ki = 0.0;

    public static final double TICKS_PER_DEGREE = 4.083; // Ticks per degree
    public static final int TURRET_TICK_LIMIT = 500; // Ticks
    public static final double FLYWHEEL_TPR = 28.0; // TPR

    // Drive constants
    public static final double NORMAL_DRIVE_RPS = 5.0;
    public static final double PRECISE_DRIVE_RPS = 0.5;

    // Declare variables
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;
    public Pose startingPose;
    public Servo turretHood;
    public Limelight3A limelight;

    // Default booleans
    public boolean isRedTeam = true;

    /**
     * Map devices; set all devices to default direction
     *
     * @param hardwareMap HardwareMap object
    */
    public HardwareController(HardwareMap hardwareMap, Pose startingPose) {
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

        // Map limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set poll rate
        limelight.setPollRateHz(100);
        limelight.start();

        // Save pose for turret localization
        this.startingPose = startingPose;

        setAllToDefault();
    }

    /**
     * Set all default directions of devices
     * Left drivetrain motors run reverse
     * Right drivetrain motors run forward
     * Default target turret rotation position (angles) to zero
    */
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
        // Set default target position
        turretYaw.setTargetPosition(0);
        turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretYaw.setPower(0.5);
    }

    /**
     * Reset turret rotation and hood position
     */
    public void resetTurret() {
        // Reset all turret actuators to default position
        turretYaw.setTargetPosition(0);
        targetSpeed = 35.0;
        turretHood.setPosition(0.5);
    }

    /**
     * Turret auto-aiming logic
     *
     * @param headingDegrees robot heading
    */
    public void autoAimTurret(double headingDegrees) {
        // Get fiducials
        List<LLResultTypes.FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        // Get team goal id
        int tID = isRedTeam ? 24 : 20;

        LLResultTypes.FiducialResult goalFiducial = null;
        boolean isGoalFound = false;
        // Loop fiducials
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            // If ID matches the goal
            if (fiducial.getFiducialId() == tID) {
                // Mark as found and break out
                isGoalFound = true;
                goalFiducial = fiducial;
                break;
            }
        }
        // If fiducial has been found, then lock on
        if (isGoalFound) {
            lockOnToGoal(goalFiducial);
        // Else, align to heading
        } else {
            alignTurretToHeading(headingDegrees);
        }
    }

    /**
     * Get the nearest valid (close) shooting location
     * Assumes the center of the field to be the origin
     *
     * @param location euclidean part of pose
     * @return closest valid shooting pose (euclidean part)
    */
    public Vector2d getShootingPose(Vector2d location) {
        int direction = (int) (location.x / Math.abs(location.x));
        // Solve the linear system of equations
        double x = direction * (location.x + location.y) / 2;
        double y = -x;
        // Return the position
        return new Vector2d(x, y);
    }

    public static double toTPS(double rps) { return rps * FLYWHEEL_TPR; }

    public void setTeam(String teamName) { isRedTeam = teamName.equals("RED"); }

    /**
     * Align to heading if turret is known not to be in frame
     *
     * @param headingDegrees robot heading
    */
    public void alignTurretToHeading(double headingDegrees) {
        // Current target position of the turret rotation (degrees)
        double position = turretYaw.getTargetPosition() / TICKS_PER_DEGREE;
        // Heading offset (degrees)
        double headingFromStart = headingDegrees - startingPose.getHeading();

        updateTurretTarget(headingFromStart - position);
    }

    /**
     * Lock on to the goal if tag is in frame
     *
     * @param fiducial fiducial output of the limelight
    */
    public void lockOnToGoal(LLResultTypes.FiducialResult fiducial) {
        // Compute distance to goal
        double tA = fiducial.getTargetArea();
        double distance = 14.76 / Math.sqrt(tA);

        // Update the turret actuators
        updateTurretTarget(fiducial.getTargetXDegrees());
        updateFlywheelByDistance(distance);
    }

    /**
     * Send the turret rotation to a certain angle (degrees)
     *
     * @param deltaAngle delta angle (degrees)
    */
    public void updateTurretTarget(double deltaAngle) {
        // Refuse commands to exceed safe rotation bounds
        if (Math.abs(deltaAngle) < 1.0) { return; }
        // Compute target position
        int deltaTicks = (int) (deltaAngle * TICKS_PER_DEGREE);
        int currentPosition = turretYaw.getCurrentPosition();
        int tempPos = currentPosition + deltaTicks;

        // Ensure new position is within the safe bounds
        targetPosition = Math.max(Math.min(tempPos, TURRET_TICK_LIMIT), -TURRET_TICK_LIMIT);
        turretYaw.setTargetPosition(targetPosition);
    }

    /**
     * Update turret position
     *
     * @param distance distance from the limelight to the goal (cm)
    */
    public void updateFlywheelByDistance(double distance) {
        // Distance must be non-negative
        if (distance < 0) {
            return;
        }
        // Compute target speed and hood angle using regression values
        targetSpeed = 0.11 * distance + 30.0;
        hoodPosition = Math.max(Math.min(0.55 - (0.00153 * distance) + (0.00000301 * Math.pow(distance, 2)), 0.5), 0.3);
        // Send values
        turretFlywheel.setVelocity(targetSpeed);
        turretHood.setPosition(hoodPosition);
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
        turretTotalError += (turretVelocityError * deltaTime); // I-value

        turretPreviousError = turretVelocityError;

        double power = (Kp * turretVelocityError) + (Kd * turretDeltaError) + (Ki * turretTotalError);
        power = Math.max(Math.min(power, 1.0), -1.0); // Limit power from -1.0 - 1.0
        return power;
    }

    /**
     * Set flywheel velocity and reset PID controller
     *
     * @param velocity target velocity for flywheel in ticks/s
     */
    public void setFlywheelVelocity(double velocity) {
        targetVelocity = velocity;
        turretTotalError = 0.0;
        turretPreviousError = 0.0;
    }
}
