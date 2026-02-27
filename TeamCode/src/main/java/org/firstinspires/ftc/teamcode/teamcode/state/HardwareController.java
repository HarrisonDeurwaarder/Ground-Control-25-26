package org.firstinspires.ftc.teamcode.teamcode.state;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class HardwareController {
    // Power constants
    public static final double INTAKE_POWER = 1.0;
    public static final double TRANSFER_POWER = 1.0;
    public static final double TURRET_ROTATION_POWER = 1.0;
    public static double DEFAULT_FLYWHEEL_RPS = 45.0; // RPS

    // Tick constants
    public static final double TURRET_ROTATION_TICKS_PER_DEGREE = 2.241; //4.317;
    public static int TURRET_POS_TICK_LIMIT = 300; // Ticks
    public static int TURRET_NEG_TICK_LIMIT = 400; // Ticks
    // 133T : 24T
    // 5.2:1 Gearbox
    // 28 Ticks/rotation motor
    public static final double FLYWHEEL_TICKS_PER_DEGREE = 0.078;
    public static final double DRIVETRAIN_TICKS_PER_DEGREE = 1.065;

    // Duration constants
    public static double FEEDING_LATENCY = 0.4; // Seconds
    public static double MICRO_TUNING_THRESHOLD = 3.0; // RPS
    public static double SHOOTING_TOLERANCE = 3.0; // Inches

    // Gate constants
    public static double GATE_OPEN_ANGLE = 0.66;
    public static double GATE_CLOSED_ANGLE = 0.5;

    // Lifting constants
    public static double LEFT_CLUTCH_DRIVE_ANGLE = 0.40;
    public static double LEFT_CLUTCH_LIFT_ANGLE = 0.500;
    public static double RIGHT_CLUTCH_DRIVE_ANGLE = 0.61;
    public static double RIGHT_CLUTCH_LIFT_ANGLE = 0.536;
    public static double LIFTING_POWER = -1.0; // RPS

    // PID
    public static PIDController flywheelMacroPID = new PIDController(0.1, 0.0, 0.0, 0.01, 0.003);
    public static PIDController flywheelMicroPID = new PIDController(0.1, 0.0, 0.0, 0.01, 0.003);
    public static PIDController turretRotationPID = new PIDController(0.01, 0.0, 0.001, 0.0, 0.05);

    // Instance variables
    public double targetSpeed = DEFAULT_FLYWHEEL_RPS;
    public double hoodPosition = 0.0;

    public double distance = 0.0;
    public Pose virtualRobotPose, virtualGoalPose;

    // Declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, flywheelA, flywheelB, turretRotation;
    public Servo turretHood, gate, clutchLeft, clutchRight;
    public Limelight3A limelight;

    // Telemetry variables
    public int turretAngle = 180;
    public int turretTicks = 0;
    public boolean tagDetected = false;

    public int manualRotationOverride = 0;
    public int liftPosition = 75;

    // Control flow flags
    public static boolean enableAutoAiming = false;
    public static boolean enableFlywheel = false;

    public static boolean enableVirtualRobotPose = false;
    public static boolean enableVirtualGoalPose = false;

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
        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        turretRotation = hardwareMap.get(DcMotorEx.class, "turretRotation");

        // Map servos
        turretHood = hardwareMap.get(Servo.class, "hood");
        gate = hardwareMap.get(Servo.class, "gate");

        clutchLeft = hardwareMap.get(Servo.class, "clutchLeft");
        clutchRight = hardwareMap.get(Servo.class, "clutchRight");

        // Map limelight
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set poll rate
        //limelight.setPollRateHz(100);
        //limelight.start();

        setAllToDefault();
    }

    /**
     * Set all default directions of devices
     * Left drivetrain motors run reverse
     * Right drivetrain motors run forward
     * Default target turret rotation position (angles) to zero
     */
    private void setAllToDefault() {
        // Set mechanism motor directions
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setDirection(DcMotorEx.Direction.REVERSE);
        turretRotation.setDirection(DcMotorEx.Direction.REVERSE);

        // Set servo directions
        turretHood.setDirection(Servo.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.5);

        clutchLeft.setDirection(Servo.Direction.FORWARD);
        clutchLeft.setPosition(LEFT_CLUTCH_DRIVE_ANGLE);
        clutchRight.setDirection(Servo.Direction.FORWARD);
        clutchRight.setPosition(RIGHT_CLUTCH_DRIVE_ANGLE);

        flywheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set turret rotation motor to use encoder
        turretRotation.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretRotation.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set default target position
        //turretRotation.setTargetPosition(0);
        //turretRotation.setPower(0.8);
        //turretRotation.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //turretRotation.setPower(TURRET_ROTATION_POWER);
    }

    /**
     * Turret auto-aiming logic
     *
     * @param follower robot follower object
     * @param goalPose goal pose
     */
    public void updateTurret(Follower follower, Pose goalPose) {

        /* VELOCITY CORRECTION */

        computeVirtualPoses(follower, goalPose);

        /* TURRET ALIGNMENT */

        // Align turret if enabled
        if (enableAutoAiming) {
            alignTurretToHeading();
        }
        // Else set to default position
        else {
            updateTurretTarget(0.0);
        }

        /* FLYWHEEL CONTROL */

        if (enableFlywheel) {
            // If flywheel enabled set parameters by distance
            double distance = virtualRobotPose.distanceFrom(virtualGoalPose);
            updateFlywheelByDistance(distance);

            sendFlywheelCommand(targetSpeed);
        } else {
            flywheelA.setPower(0.0);
            flywheelB.setPower(0.0);
            // Default hood position
            hoodPosition = 0.0;
        }
        // Set hood position
        turretHood.setPosition(hoodPosition);
    }

    /**
     * High-level send a velocity to the flywheel
     *
     * @param velocity target velocity in RPS
     */
    public void sendFlywheelCommand(double velocity) {
        double flywheelVel = flywheelA.getVelocity() / (360 * FLYWHEEL_TICKS_PER_DEGREE);
        // Select fine-adjustment pid if necessary
        PIDController pid = Math.abs(velocity - flywheelVel) <= MICRO_TUNING_THRESHOLD ? flywheelMicroPID : flywheelMacroPID;
        double power = pid.compute(velocity, flywheelVel);
        // Set the flywheel power
        flywheelA.setPower(
                Math.max(-0.7, Math.min(power, 1.0))
        );
        // Set the flywheel power
        flywheelB.setPower(
                Math.max(-0.7, Math.min(power, 1.0))
        );
    }

    public void startLift(double timeSinceLiftStarted) {
        // Activate the clutch
        clutchLeft.setPosition(LEFT_CLUTCH_LIFT_ANGLE);
        clutchRight.setPosition(RIGHT_CLUTCH_LIFT_ANGLE);
        // Continue if servos are in correct position
        if (Math.abs(clutchLeft.getPosition() - LEFT_CLUTCH_LIFT_ANGLE) <= 0.1 && Math.abs(clutchRight.getPosition() - RIGHT_CLUTCH_LIFT_ANGLE) <= 0.1) {
            // Set motor positions incrementally
            leftFront.setTargetPosition(liftPosition);
            rightFront.setTargetPosition(liftPosition);
            // Increment
            if (Math.abs(leftFront.getCurrentPosition() - liftPosition) <= 5 && Math.abs(rightFront.getCurrentPosition() - liftPosition) <= 5) liftPosition += 75;
        }
    }

    /**
     * Send the turret rotation to a certain angle while maintaining the rotational bounds
     *
     * @param angle angle (degrees)
     */
    public void updateTurretTarget(double angle) {
        int ticks180 = (int) (180 * TURRET_ROTATION_TICKS_PER_DEGREE);
        // Compute target position
        int ticks = (int) (angle * TURRET_ROTATION_TICKS_PER_DEGREE) + manualRotationOverride;

        // Modify tick count to stay in bounds
        if (ticks < -TURRET_NEG_TICK_LIMIT) {
            ticks = ticks180 - (-ticks % ticks180);
        } else if (ticks > TURRET_POS_TICK_LIMIT) {
            ticks %= ticks180;
            ticks -= ticks180;
        }

        // Final cut to ensure bounds are met
        turretTicks = Math.max(-TURRET_NEG_TICK_LIMIT, Math.min(ticks, TURRET_POS_TICK_LIMIT));
        turretRotation.setPower(
                Math.max(-1.0, Math.min(turretRotationPID.compute(turretTicks, turretRotation.getCurrentPosition()), 1.0))
        );
    }

    public boolean inShootingZone(Follower follower) {
        Pose pose = follower.getPose();
        // Check close & far positions
        return (pose.getY() + SHOOTING_TOLERANCE >= Math.abs(pose.getX())) || (pose.getY() - SHOOTING_TOLERANCE + 48.0 <= -Math.abs(pose.getX()));
    }

    public Pose getNearestShootingPose(Follower follower) {
        Pose pose = follower.getPose();
        double Px = pose.getX();
        double Py = pose.getY();

        Pose nearPos;
        // Far zone
        Pose farPos = new Pose(0.0, -48.0);

        // Red side
        if (pose.getX() >= 0.0) {
            // Near zone
            nearPos = new Pose(
                    (Px + Py) / 2.0,
                    (Px + Py) / 2.0
            );
            // Apply bounds
            if (nearPos.getX() > 44.0) nearPos = new Pose(44.0, 44.0);
            if (nearPos.getX() < 0.0) nearPos = new Pose(0.0, 0.0);

        // Blue side
        } else {
            // Near zone
            nearPos = new Pose(
                    (Px - Py) / 2.0,
                    (Py - Px) / 2.0
            );
            // Apply bounds
            if (nearPos.getX() < -44.0) nearPos = new Pose(-44.0, 44.0);
            if (nearPos.getX() > 0.0) nearPos = new Pose(0.0, 0.0);
        }

        // Extract closest pose
        return nearPos.distanceFrom(pose) >= farPos.distanceFrom(pose) ? nearPos : farPos;
    }

    /**
     * Align the turret by heading
    */
    private void alignTurretToHeading() {
        // Computes the robot->goal vector
        Vector goalPosition = new Vector(virtualGoalPose);
        Vector robotPosition = new Vector(virtualRobotPose);
        Vector goalFromRobot = goalPosition.minus(robotPosition);
        // Robot heading offset
        int headingShifted = (int) (Math.toDegrees(Math.atan2(goalFromRobot.getYComponent(), goalFromRobot.getXComponent())));
        int headingOffset = headingShifted - 90;
        turretAngle = headingOffset - (int)(Math.toDegrees(virtualRobotPose.getHeading()));

        updateTurretTarget(turretAngle);
    }

    /**
     * Compute the effective target for turret alignment
     *
     * @param goalPose goal pose
     */
    private void computeVirtualPoses(Follower follower, Pose goalPose) {
        /* VIRTUAL ROBOT POSE */

        virtualRobotPose = enableVirtualRobotPose ? new Pose(
                follower.getPose().getX() + FEEDING_LATENCY * follower.getVelocity().getXComponent() + (Math.pow(FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getXComponent(),
                follower.getPose().getY() + FEEDING_LATENCY * follower.getVelocity().getYComponent() + (Math.pow(FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getYComponent(),
                follower.getHeading() + FEEDING_LATENCY * follower.getAngularVelocity()
        ) : follower.getPose();

        /* VIRTUAL GOAL POSE */

        virtualGoalPose = enableVirtualGoalPose ? new Pose(
                goalPose.getX() - computeAirtime(virtualRobotPose, goalPose) * (follower.getVelocity().getXComponent() + FEEDING_LATENCY * follower.getAcceleration().getXComponent()),
                goalPose.getY() - computeAirtime(virtualRobotPose, goalPose) * (follower.getVelocity().getYComponent() + FEEDING_LATENCY * follower.getAcceleration().getYComponent())
        ) : goalPose;
    }

    /**
     * Update flywheel speed by regression values
    */
    private void updateFlywheelByDistance(double distance) {
        // Compute flywheel velocity
        // Linear interp
        if (distance <= 70) {
            targetSpeed = 0.2 * distance + 32.0;
        } else if (distance <= 80) {
            targetSpeed = 0.3 * distance + 25.0;
        } else if (distance <= 90) {
            targetSpeed = 0.2 * distance + 33.0;
        } else if (distance <= 120) {
            targetSpeed = 0.23 * distance + 29.0;
        } else if (distance <= 130) {
            targetSpeed = 0.3 * distance + 21.0;
        } else {
            targetSpeed = 0.2 * distance + 34.0;
        }

        // Compute hood angle
        if (distance <= 30) {
            hoodPosition = 0.0;
        } else if (distance <= 40) {
            hoodPosition = 0.04 * distance - 1.2;
        } else if (distance <= 90) {
            hoodPosition = 0.01 * distance - 0.0;
        } else {
            hoodPosition = 0.9;
        }
    }

    private double computeAirtime(Pose robotPose, Pose goalPose) { return 0.0479 * robotPose.distanceFrom(goalPose) + 0.676; }

    // Hood Angle: 0.00438x + 0.0457
    // Flywheel Speed (RPS): 0.176x + 33.9
}