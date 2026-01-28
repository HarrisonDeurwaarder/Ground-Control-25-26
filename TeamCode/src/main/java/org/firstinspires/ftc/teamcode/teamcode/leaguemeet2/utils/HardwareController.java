package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.acmerobotics.roadrunner.Vector2d;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.KeyStore;
import java.util.List;

@Configurable
public class HardwareController {
    // Set constants
    public static final double INTAKE_POWER = 0.8;
    public static final double TRANSFER_POWER = 0.5;

    public static final double DEFAULT_FLYWHEEL_RPS = 45.0; // RPS
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
    public double Kp = 0.5;
    public double Kd = 0.0;
    public double Ki = 0.0;
    public double distance = 0.0;
    public boolean autoAiming = true;

    public static final double TICKS_PER_DEGREE = 6.806; // Ticks per degree
    public static final int TURRET_TICK_LIMIT = 1200; // Ticks
    public static final double FLYWHEEL_TPR = 28.0; // TPR

    // Declare variables
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;
    public Pose startingPose;
    public Servo turretHood;
    public Limelight3A limelight;

    public int turretAngle = 180;
    public int turretTicks = 0;
    public boolean tagDetected = false;

    // Default booleans
    public boolean isRedTeam = true;
    public LLResultTypes.FiducialResult goalFiducial = null;


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
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        turretFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        turretYaw.setDirection(DcMotorEx.Direction.REVERSE);

        // Set servo direction
        turretHood.setDirection(Servo.Direction.FORWARD);

        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Set turret yaw motor to use encoder
        turretYaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretYaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set default target position
        turretYaw.setTargetPosition(0);
        turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretYaw.setPower(0.8);
    }

    /**
     * Reset turret rotation and hood position
     */
    public void resetTurret() {
        // Reset all turret actuators to default position
        updateTurretTarget(0);
        targetSpeed = 35.0;
        turretHood.setPosition(0.5);
    }

    /**
     * Turret auto-aiming logic
     *
     * @param pose robot pose
    */
    public void autoAimTurret(Pose pose) {
        // Get fiducials
        List<LLResultTypes.FiducialResult> fiducials = limelight.getLatestResult().getFiducialResults();
        // Get team goal id
        int tID = isRedTeam ? 24 : 20;

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
        tagDetected = isGoalFound;
        if (goalFiducial != null) {
            lockOnToGoal(goalFiducial);
        }
        // Always align to heading
        if (autoAiming) {
            alignTurretToHeading(pose);
        }
        else {
            turretYaw.setTargetPosition((int) (-90.0 * TICKS_PER_DEGREE));
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
     * Returns the angle of a vector from the x-axis in degrees. Values are bounded to positive angles
    */
    private int getTheta(Vector vector) {
        double theta = Math.atan2(vector.getYComponent(), vector.getXComponent());
        // Convert theta to degrees
        theta = Math.toDegrees(theta);
        return (int) theta;
    }

    /**
     * Align to heading if goal is known not to be in frame
     *
     * @param pose robot pose
    */
    private void alignTurretToHeading(Pose pose) {
        // Goal offset from robot
        // Accounts for correct goal location
        Vector goalPosition = new Vector((isRedTeam) ? new Pose(60.0, 60.0) : new Pose(-60.0, 70.0));
        Vector robotPosition = new Vector(pose);
        Vector goalFromRobot = goalPosition.minus(robotPosition);
        // Robot heading offset
        int headingOffset = getTheta(goalFromRobot) - 90;
        turretAngle = headingOffset - (int)(Math.toDegrees(pose.getHeading()));

        //turretYaw.setTargetPosition((int) (headingOffset * TICKS_PER_DEGREE));

        updateTurretTarget(turretAngle);
    }

    /**
     * Lock on to the goal if tag is in frame
     *
     * @param fiducial fiducial output of the limelight
    */
    private void lockOnToGoal(LLResultTypes.FiducialResult fiducial) {
        // Compute distance to goal
        double tA = fiducial.getTargetArea();
        distance = 14.76 / Math.sqrt(tA);

        // Update the turret actuators
        // updateTurretTarget(fiducial.getTargetXDegrees());
        updateFlywheelByDistance();
    }

    /**
     * Send the turret rotation to a certain angle
     *
     * @param angle angle (degrees)
    */
    public void updateTurretTarget(double angle) {
        int ticks360 = (int) (360 * TICKS_PER_DEGREE);
        int ticks180 = (int) (180 * TICKS_PER_DEGREE);
        // Compute target position
        int ticks = (int) (angle * TICKS_PER_DEGREE);

        // Modify tick count to stay in bounds

        if (ticks < -TURRET_TICK_LIMIT) {
            ticks = ticks180 - (-ticks % ticks180);
        } else if (ticks > TURRET_TICK_LIMIT) {
            ticks %= ticks180;
            ticks -= ticks180;
        }

        //ticks = (ticks + ticks180) % ticks180 - ticks180;
        // Final cut to ensure bounds are met
        turretTicks = Math.max(-TURRET_TICK_LIMIT, Math.min(ticks, TURRET_TICK_LIMIT));
        turretYaw.setTargetPosition(turretTicks);
    }

    /**
     * Update turret position
     *
    */
    public void updateFlywheelByDistance() {
        // Distance must be non-negative
        if (distance < 0) {
            return;
        }
        // Compute target speed and hood angle using regression values
        targetSpeed = 0.11 * distance + 31.5;
        hoodPosition = Math.max(Math.min(0.55 - (0.00153 * distance) + (0.00000301 * Math.pow(distance, 2)), 0.50), 0.3);
        // Send values
        turretFlywheel.setPower(PID_Controller());
        //turretFlywheel.setVelocity(targetSpeed);
        turretHood.setPosition(hoodPosition);
    }

    /**
     * PID controller for turret velocity
     *
     * @return power output power for flywheel motor
     */
    public double PID_Controller() {
        turretVelocityError = targetSpeed - (turretFlywheel.getVelocity() / FLYWHEEL_TPR); // P-value

        double power = (Kp * turretVelocityError);
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
