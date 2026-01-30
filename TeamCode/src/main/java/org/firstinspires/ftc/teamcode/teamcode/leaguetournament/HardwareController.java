package org.firstinspires.ftc.teamcode.teamcode.leaguetournament;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.List;

@Config
public class HardwareController {
    // Power constants
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_POWER = 1.0;
    public static final double TURRET_ROTATION_POWER = 0.8;

    // Miscellaneous constants
    public static final double TURRET_ROTATION_TICKS_PER_DEGREE = 4.317;
    public static final double FLYWHEEL_TICKS_PER_DEGREE = 0.077;
    public static final int TURRET_TICK_LIMIT = 800; // Ticks

    public static double DEFAULT_FLYWHEEL_RPS = 45.0; // RPS
    public static double ARTIFACT_AIRTIME = 0.5; // Seconds

    public static double OPEN_ANGLE   = 0.71;
    public static double CLOSED_ANGLE = 0.61;

    // Adaptive turret variables
    public double targetSpeed = DEFAULT_FLYWHEEL_RPS;
    public double hoodPosition = 0.5;

    public double distance = 0.0;
    public double lastRecordedError = 0.0;
    public double lastRecordedTime = 0.0;

    // Flywheel PID
    public static double Kp = 0.5;
    public static double Kd = 0.0003;
    public static double Ks = 0.2;

    // Declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretRotation;
    public Servo turretHood, gate;
    public Limelight3A limelight;

    // Telemetry variables
    public int turretAngle = 180;
    public int turretTicks = 0;
    public boolean tagDetected = false;

    // Control flow flags
    public boolean enableAutoAiming = true;
    public boolean enableFlywheel = true;
    public boolean enableArtifactVelocityCorrection = false;

    /**
     * Map devices; set all devices to default direction
     *
     * @param hardwareMap HardwareMap object
    */
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
        turretRotation = hardwareMap.get(DcMotorEx.class, "turretRotation");

        // Map servos
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        gate = hardwareMap.get(Servo.class, "turretGate");

        // Map limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set poll rate
        limelight.setPollRateHz(100);
        limelight.start();

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
        transfer.setDirection(DcMotorEx.Direction.REVERSE);
        turretFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        turretRotation.setDirection(DcMotorEx.Direction.FORWARD);

        // Set servo directions
        turretHood.setDirection(Servo.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.6);

        turretFlywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set turret rotation motor to use encoder
        turretRotation.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set default target position
        turretRotation.setTargetPosition(0);
        turretRotation.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretRotation.setPower(TURRET_ROTATION_POWER);
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
     * @param follower robot follower object
     * @param goalPose goal pose
    */
    public void updateTurret(Follower follower, Pose goalPose, double timestamp) {
        // Translate the goal pose in accordance with expected velocity if enabled
        if (enableArtifactVelocityCorrection) goalPose = new Pose(
                    goalPose.getX() - ARTIFACT_AIRTIME * follower.getVelocity().getXComponent(),
                    goalPose.getY() - ARTIFACT_AIRTIME * follower.getVelocity().getYComponent()
            );

        // Align turret if enabled
        if (enableAutoAiming) {
            alignTurretToHeading(follower.getPose(), goalPose);
        }
        // Else set to default position
        else {
            turretRotation.setTargetPosition((int) (-90.0 * TURRET_ROTATION_TICKS_PER_DEGREE));
        }

        // Control flywheel if enabled
        if (enableFlywheel) {
            // If flywheel enabled set parameters by distance
            double distance = follower.getPose().distanceFrom(goalPose);
            updateFlywheelByDistance(distance);
        // Else set to zero velocity
        } else {
            targetSpeed = 0.0;
        }
        // Set the flywheel power
        PDController(timestamp - lastRecordedTime);
        lastRecordedTime = timestamp;
    }

    /**
     * Send the turret rotation to a certain angle while maintaining the rotational bounds
     *
     * @param angle angle (degrees)
     */
    public void updateTurretTarget(double angle) {
        int ticks180 = (int) (180 * TURRET_ROTATION_TICKS_PER_DEGREE);
        // Compute target position
        int ticks = (int) (angle * TURRET_ROTATION_TICKS_PER_DEGREE);

        // Modify tick count to stay in bounds
        if (ticks < -TURRET_TICK_LIMIT) {
            ticks = ticks180 - (-ticks % ticks180);
        } else if (ticks > TURRET_TICK_LIMIT) {
            ticks %= ticks180;
            ticks -= ticks180;
        }

        // Final cut to ensure bounds are met
        turretTicks = Math.max(-TURRET_TICK_LIMIT, Math.min(ticks, TURRET_TICK_LIMIT));
        turretRotation.setTargetPosition(turretTicks);
    }

    /**
     * Align the turret by heading
     *
     * @param robotPose robot pose
     * @param goalPose goal pose
    */
    private void alignTurretToHeading(Pose robotPose, Pose goalPose) {
        // Computes the robot->goal vector
        Vector goalPosition = new Vector(goalPose);
        Vector robotPosition = new Vector(robotPose);
        Vector goalFromRobot = goalPosition.minus(robotPosition);
        // Robot heading offset
        int headingShifted = (int) (Math.toDegrees(Math.atan2(goalFromRobot.getYComponent(), goalFromRobot.getXComponent())));
        int headingOffset = headingShifted - 90;
        turretAngle = headingOffset - (int)(Math.toDegrees(robotPose.getHeading()));

        updateTurretTarget(turretAngle);
    }

    /**
     * Update flywheel speed by regression values
    */
    private void updateFlywheelByDistance(double distance) {
        // Compute distance to goal
        // Compute target speed and hood angle using regression values
        targetSpeed = Math.min(0.176 * distance + 33.9, 58); // +1 is for diff in target/actual speed
        hoodPosition = Math.max(Math.min((0.00438 * distance + 0.0457), 0.57), 0.19);
        // Send values
        //turretFlywheel.setVelocity(targetSpeed);
        turretHood.setPosition(hoodPosition);
    }



    /**
     * PD controller for turret velocity
     *
     * @param deltaTime time since last run cycle
     */
    public void PDController(double deltaTime) {
        double proportional = targetSpeed - (turretFlywheel.getVelocity() / 28.0);
        double derivative   = (proportional - lastRecordedError) / deltaTime;
        double constant     = Math.signum(proportional);
        // Compute power
        double power = Kp * proportional + Kd * derivative + Ks * constant;
        // Clip power
        power = Math.max(Math.min(power, 1.0), -1.0);
        turretFlywheel.setPower(power);

        // Save previous
        lastRecordedError = proportional;
    }

    // Hood Angle: 0.00438x + 0.0457
    // Flywheel Speed (RPS): 0.176x + 33.9
}