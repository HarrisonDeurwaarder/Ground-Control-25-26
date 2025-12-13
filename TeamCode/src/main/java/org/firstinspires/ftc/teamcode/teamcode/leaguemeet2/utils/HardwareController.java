package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;

public class HardwareController {
    // Set constants
    public static final double INTAKE_POWER = 0.5;
    public static final double TRANSFER_POWER = 0.5;

    public static final double FLYWHEEL_RPS = 2.3; // RPS
    public static final double FLYWHEEL_TPS = toTPS(FLYWHEEL_RPS); // TPS
    public static final double FW_VEL_ERROR = 0.15; // RPS

    public static final double TURRET_ALIGNMENT_ERROR = 0.05; // Meters
    public static final double TURRET_REVOLUTION_TOLERANCE = Math.PI / 12; // Radians
    public static final int SEARCH_INCREMENT = 10; // Ticks

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
        turretYaw = hardwareMap.get(DcMotorEx.class, "turretFlywheel");

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
                isOn ? FLYWHEEL_TPS : 0.0
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
        boolean flywheelInRange = getFlywheelVelocity() < FLYWHEEL_RPS + FW_VEL_ERROR && getFlywheelVelocity() > FLYWHEEL_RPS - FW_VEL_ERROR;
        // Feed only if flywheel is in acceptable velocity range
        if (flywheelInRange) {
            transfer.setPower(TRANSFER_POWER);
        } else {
            transfer.setPower(0.0);
        }
        // Return result for external use
        return flywheelInRange;
    }

    /**
     * Converts a velocity in rotations-per-second to ticks-per-second
     *
     * @param rps rotations per second
     * @return ticks per second
    */
    public static double toTPS(double rps) { return rps * GOBILDA_TPR; }

    /**
     * Converts a velocity in ticks-per-second to rotations-per-second
     *
     * @param tps ticks per second
     * @return rotations per second
     */
    public static double toRPS(double tps) { return tps / GOBILDA_TPR; }

    /**
     * Unless it is already aligned, continuously sends the proper velocities to align turret
     * Assumes that the goal is in frame
     *
     * @param finderOutput fiducial result of the goal detection
     * @return if the position has been reached (in error range)
    */
    public boolean alignTurretToGoal(LLResultTypes.FiducialResult finderOutput) {
        // If turret is already properly aligned, skip logic
        Pose3D targetPose = finderOutput.getTargetPoseCameraSpace();
        if (!turretInAcceptableRange(targetPose.getPosition())) {
            // Set desired position
            double yawOffsetRads = targetPose.getOrientation().getYaw(AngleUnit.RADIANS);
            updateTurretYawTarget(
                    (int) ((yawOffsetRads * GOBILDA_TPR) / (Math.PI * 2)) // Convert radians to ticks
            );
            // Position has not yet been reached
            return false;
        }
        // Position has been reached
        return true;
    }

    /**
     * Searching logic when the goal is not in frame
    */
    public void searchForGoal() {
        // Change the sign of the search increment based on the current velocity
        // If position overflows, positive values will not continue compounding
        updateTurretYawTarget(
                (turretYaw.getVelocity() > 0 ? SEARCH_INCREMENT : -SEARCH_INCREMENT)
        );
    }

    /**
     * Adds a given tick increment to the turret yaw motor; stays in a single 2*PI radian arc (+ 2*TOLERANCE)
     *
     * @param tickIncrement amount to add to the target position
    */
    private void updateTurretYawTarget(int tickIncrement) {
        // Calculate the new target position
        // Using current position as opposed to target position to increment proves more useful for turret alignment
        int newTarget = turretYaw.getCurrentPosition() + tickIncrement;

        // Roll back if it exceeds the error range
        // In positive direction (PI + TOLERANCE rads)
        if (newTarget > (GOBILDA_TPR / 2) + TURRET_REVOLUTION_TOLERANCE_TICKS) {
            newTarget %= (int) (GOBILDA_TPR);
        // In negative direction (-PI - TOLERANCE rads)
        } else if (newTarget < -((GOBILDA_TPR / 2) + TURRET_REVOLUTION_TOLERANCE_TICKS)) {
            newTarget %= (int) (GOBILDA_TPR);
            // Keep negative direction
            newTarget -= (int) (GOBILDA_TPR);
        }
        turretYaw.setTargetPosition(newTarget);
    }

    /**
     * Checks if the position offset is within the error range
     *
     * @param pos offset of camera to tag in euclidean space
     * @return if the position is in range
    */
    private static boolean turretInAcceptableRange(Position pos) {
        // Verify that the position is within the allowed range
        return pos.y <= TURRET_ALIGNMENT_ERROR && pos.y >= -TURRET_ALIGNMENT_ERROR;
    }
}
