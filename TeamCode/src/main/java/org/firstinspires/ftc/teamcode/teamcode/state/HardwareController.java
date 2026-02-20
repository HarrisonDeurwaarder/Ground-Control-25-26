package org.firstinspires.ftc.teamcode.teamcode.state;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
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
    public static final double TURRET_ROTATION_TICKS_PER_DEGREE = 4.317;
    public static final double FLYWHEEL_TICKS_PER_DEGREE = 0.077;
    public static final int TURRET_TICK_LIMIT = 800; // Ticks

    // Duration constants
    public static double ARTIFACT_AIRTIME = 0.7; // Seconds
    public static double FEEDING_THROUGHPUT = 0.5; // Seconds

    // Transfer constants
    public static double OPEN_ANGLE = 0.81;
    public static double CLOSED_ANGLE = 0.6;

    // PID
    public static PIDController flywheelPID = new PIDController(0.5, 0.0, 0.0003, 0.0, 0.2);

    // Instance variables
    public double targetSpeed = DEFAULT_FLYWHEEL_RPS;
    public double hoodPosition = 0.5;

    public double distance = 0.0;
    public Pose virtualRobotPose, virtualGoalPose;

    // Declare actuators
    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, flywheelA, flywheelB, turretRotation;
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
        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        turretRotation = hardwareMap.get(DcMotorEx.class, "turretRotation");

        // Map servos
        turretHood = hardwareMap.get(Servo.class, "turretHood");
        gate = hardwareMap.get(Servo.class, "gate");

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
        flywheelA.setDirection(DcMotorEx.Direction.REVERSE);
        flywheelB.setDirection(DcMotorEx.Direction.REVERSE);
        turretRotation.setDirection(DcMotorEx.Direction.FORWARD);

        // Set servo directions
        turretHood.setDirection(Servo.Direction.FORWARD);
        gate.setDirection(Servo.Direction.FORWARD);
        gate.setPosition(0.6);

        flywheelA.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelA.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        flywheelB.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flywheelB.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set turret rotation motor to use encoder
        turretRotation.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretRotation.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // Set default target position
        turretRotation.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(5.0, 1.0, 0.0, 0.0));
        turretRotation.setTargetPosition(0);
        turretRotation.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //turretRotation.setPower(TURRET_ROTATION_POWER);
    }

    /**
     * Turret auto-aiming logic
     *
     * @param follower robot follower object
     * @param goalPose goal pose
     */
    public void updateTurret(Follower follower, Pose goalPose, double timestamp) {
        /* VELOCITY CORRECTION */

        // Translate the goal pose in accordance with expected velocity if enabled
        if (enableArtifactVelocityCorrection) goalPose = new Pose(
                goalPose.getX() - ARTIFACT_AIRTIME * follower.getVelocity().getXComponent(),
                goalPose.getY() - ARTIFACT_AIRTIME * follower.getVelocity().getYComponent()
        );

        /* TURRET ALIGNMENT */

        // Align turret if enabled
        if (enableAutoAiming) {
            computeVirtualPoses(follower.getPose(), goalPose);
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
            // Else set to zero velocity
        } else {
            targetSpeed = 0.0;
        }
        // Set the flywheel power
        flywheelA.setPower(
                Math.max(-1.0, Math.min(flywheelPID.compute(
                        targetSpeed, flywheelA.getVelocity()
                ), 1.0))
        );
        // Set the flywheel power
        flywheelB.setPower(
                Math.max(-1.0, Math.min(flywheelPID.compute(
                        targetSpeed, flywheelA.getVelocity()
                ), 1.0))
        );
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
    private void computeVirtualPoses(Pose robotPose, Pose goalPose) {
        /* VIRTUAL ROBOT POSE */
        /* VIRTUAL GOAL POSE */
    }

    /**
     * Update flywheel speed by regression values
    */
    private void updateFlywheelByDistance(double distance, double y) {
        // Compute distance to goal
        // Compute target speed and hood angle using regression values
        if (y >= -12) {
            targetSpeed = Math.min(0.259 * distance + 29.0, 60); // +1 is for diff in target/actual speed}
            hoodPosition = Math.max(Math.min((0.00296 * distance + 0.107), 0.5), 0.19);
        }
        else {
            targetSpeed = 57.0;
            hoodPosition = 0.45;
        }
            // Send values
            //turretFlywheel.setVelocity(targetSpeed);
            turretHood.setPosition(hoodPosition);
    }

    // Hood Angle: 0.00438x + 0.0457
    // Flywheel Speed (RPS): 0.176x + 33.9
}


class PIDController {
    public double Kp, Ki, Kd, Kf, Ks;
    private double lastError, lastTimestamp, integral = 0.0;
    private Timer plantTimer;

    public PIDController(double p, double i, double d, double f, double s) {
        setCoefficients(p, i, d, f, s);
        plantTimer = new Timer();
        plantTimer.resetTimer();
    }

    public PIDController(double beta, double tau, double K, double lambda) {
        setCoefficients(beta, tau, K, lambda);
        plantTimer = new Timer();
        plantTimer.resetTimer();
    }

    /**
     * Set the coefficients of the controller
     *
     * @param p proportional coefficient
     * @param i integral coefficient
     * @param d derivative coefficient
     * @param f feedforward coefficient
     * @param s static coefficient
     */
    public void setCoefficients(double p, double i, double d, double f, double s) {
        // Map coefficients
        Kp = p; Ki = i; Kd = d; Kf = f; Ks = s;
    }

    /**
     * Set the coefficients of the controller (by lambda constants)
     *
     * @param beta figure
     * @param tau it
     * @param K out
     * @param lambda ;)
     */
    public void setCoefficients(double beta, double tau, double K, double lambda) {
        // Compute PID coefficients from lambda-tuned constants
        Kp = ((1.0 + beta) * tau) / (K * lambda);
        Kd = beta * tau / K;
        // Lambda tuned PIDs only use PD
        Ki = 0.0; Kf = 0.0; Ks = 0.0;
    }

    public double compute(double setpoint, double measurement) {
        double deltaTime = plantTimer.getElapsedTimeSeconds() - lastTimestamp;
        lastTimestamp = plantTimer.getElapsedTimeSeconds();

        // Compute PID
        double error = setpoint - measurement;
        double derivative = (error - lastError) / deltaTime;
        integral += error * deltaTime;
        lastError = error;

        return Kp * error + Ki * integral + Kd * derivative + Kf * setpoint + Ks * Math.signum(error);
    }
}