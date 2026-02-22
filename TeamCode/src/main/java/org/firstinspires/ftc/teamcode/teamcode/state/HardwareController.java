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
    public static final int TURRET_TICK_LIMIT = 400; // Ticks
    // 133T : 24T
    // 5.2:1 Gearbox
    // 28 Ticks/rotation motor
    public static final double FLYWHEEL_TICKS_PER_DEGREE = 0.078;
    public static final double DRIVETRAIN_TICKS_PER_DEGREE = 1.065;

    // Duration constants
    public static double ARTIFACT_AIRTIME = 0.7; // Seconds
    public static double FEEDING_LATENCY = 0.5; // Seconds
    public static double MICRO_TUNING_THRESHOLD = 3.0; // RPS

    // Gate constants
    public static double GATE_OPEN_ANGLE = 0.66;
    public static double GATE_CLOSED_ANGLE = 0.5;

    // Lifting constants
    public static double CLUTCH_DRIVE_ANGLE = 0.0;
    public static double CLUTCH_LIFT_ANGLE = 0.0;
    public static double LOCK_DRIVE_ANGLE = 0.0;
    public static double LOCK_LIFT_ANGLE = 0.0;
    public static double LIFTING_VELOCITY = 1.0; // RPS

    // PID
    public static PIDController flywheelMacroPID = new PIDController(0.1, 0.0, 0.0, 0.01, 0.003);
    public static PIDController flywheelMicroPID = new PIDController(0.1, 0.0, 0.0, 0.01, 0.003);

    // Instance variables
    public double targetSpeed = DEFAULT_FLYWHEEL_RPS;
    public double hoodPosition = 0.0;

    public double distance = 0.0;
    public Pose virtualRobotPose, virtualGoalPose;

    // Declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, flywheelA, flywheelB, turretRotation;
    public Servo turretHood, gate, clutchLeft, clutchRight, lock;
    public Limelight3A limelight;

    // Telemetry variables
    public int turretAngle = 180;
    public int turretTicks = 0;
    public boolean tagDetected = false;

    // Control flow flags
    public static boolean enableArtifactVelocityCorrection = false;
    public static boolean enableAutoAiming = false;
    public static boolean enableFlywheel = false;

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
        lock = hardwareMap.get(Servo.class, "lock");

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
        // Set drivetrain motor directions
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.REVERSE);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.REVERSE);

        // Set mechanism motor directions
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setDirection(DcMotorEx.Direction.REVERSE);
        turretRotation.setDirection(DcMotorEx.Direction.REVERSE);

        // Set servo directions
        turretHood.setDirection(Servo.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.5);

        //clutchLeft.setDirection(Servo.Direction.FORWARD);
        //clutchLeft.setPosition(CLUTCH_DRIVE_ANGLE);
        //clutchRight.setDirection(Servo.Direction.FORWARD);
        //clutchRight.setPosition(CLUTCH_DRIVE_ANGLE);
        //lock.setDirection(Servo.Direction.FORWARD);
        //lock.setPosition(LOCK_DRIVE_ANGLE);

        flywheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set turret rotation motor to use encoder
        turretRotation.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set default target position
        turretRotation.setTargetPosition(0);
        turretRotation.setPower(0.8);
        turretRotation.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
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

        // Translate the goal pose in accordance with expected velocity if enabled
        if (enableArtifactVelocityCorrection) goalPose = new Pose(
                goalPose.getX() - ARTIFACT_AIRTIME * follower.getVelocity().getXComponent(),
                goalPose.getY() - ARTIFACT_AIRTIME * follower.getVelocity().getYComponent()
        );

        /* VELOCITY CORRECTION */

        if (enableArtifactVelocityCorrection) {
            computeVirtualPoses(follower, goalPose);
        } else {
            virtualRobotPose = follower.getPose();
            virtualGoalPose = goalPose;
        }

        /* TURRET ALIGNMENT */

        // Align turret if enabled
        if (enableAutoAiming) {
            alignTurretToHeading();
        }
        // Else set to default position
        else {
            turretRotation.setTargetPosition((int) (-90.0 * TURRET_ROTATION_TICKS_PER_DEGREE));
        }

        /* FLYWHEEL CONTROL */

        if (enableFlywheel) {
            // If flywheel enabled set parameters by distance
            double distance = follower.getPose().distanceFrom(goalPose);
            updateFlywheelByDistance(distance, follower.getPose().getY());

            sendFlywheelCommand(targetSpeed);
        } else {
            flywheelA.setPower(0.0);
            flywheelB.setPower(0.0);
        }
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

    public void startLift() {
        // Activate the clutch
        //clutchLeft.setPosition(CLUTCH_LIFT_ANGLE);
        //clutchRight.setPosition(CLUTCH_LIFT_ANGLE);
        // Lock the plate
        //lock.setPosition(LOCK_LIFT_ANGLE);
        // Drive motors
        leftFront.setVelocity(LIFTING_VELOCITY * DRIVETRAIN_TICKS_PER_DEGREE * 360.0);
        leftBack.setVelocity(LIFTING_VELOCITY * DRIVETRAIN_TICKS_PER_DEGREE * 360.0);
        rightFront.setVelocity(LIFTING_VELOCITY * DRIVETRAIN_TICKS_PER_DEGREE * 360.0);
        rightBack.setVelocity(LIFTING_VELOCITY * DRIVETRAIN_TICKS_PER_DEGREE * 360.0);
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

        virtualRobotPose = new Pose(
                follower.getPose().getX() + FEEDING_LATENCY * follower.getVelocity().getXComponent() + (Math.pow(FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getXComponent(),
                follower.getPose().getY() + FEEDING_LATENCY * follower.getVelocity().getYComponent() + (Math.pow(FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getYComponent(),
                follower.getHeading() + FEEDING_LATENCY * follower.getAngularVelocity()
        );

        /* VIRTUAL GOAL POSE */

        virtualGoalPose = new Pose(
                goalPose.getX() - computeAirtime(follower.getPose(), goalPose) * (follower.getVelocity().getXComponent() + FEEDING_LATENCY * follower.getAcceleration().getXComponent()),
                goalPose.getY() - computeAirtime(follower.getPose(), goalPose) * (follower.getVelocity().getYComponent() + FEEDING_LATENCY * follower.getAcceleration().getYComponent())
        );
    }

    /**
     * Update flywheel speed by regression values
    */
    private void updateFlywheelByDistance(double distance, double y) {
        // Compute distance to goal
        // Compute target speed and hood angle using regression values
        if (y >= -12) {
            targetSpeed = 50.0; // Math.min(0.259 * distance + 29.0, 60); // +1 is for diff in target/actual speed}
            hoodPosition = 0.5; // Math.max(Math.min((0.00296 * distance + 0.107), 0.5), 0.19);
        }
        else {
            targetSpeed = 50.0;
            hoodPosition = 0.5;
        }
            // Send values
            //turretFlywheel.setVelocity(targetSpeed);
            turretHood.setPosition(hoodPosition); //
    }

    private double computeAirtime(Pose robotPose, Pose goalPose) { return 0.0 * robotPose.distanceFrom(goalPose) + 0.5; }

    // Hood Angle: 0.00438x + 0.0457
    // Flywheel Speed (RPS): 0.176x + 33.9
}