package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HardwareController {
    // Set constants
    public static final double INTAKE_RPS = 2.0;
    public static final double TRANSPORT_RPS = 2.0;
    public static final double FLYWHEEL_RPS = 4.25;

    public static final double TRIGGER_THRESHOLD = 0.05;
    public static final double FW_VEL_ERROR = 0.2;

    public static final double NORMAL_DRIVE_RPS = 5.0;
    public static final double PRECISE_DRIVE_RPS = 0.5;

    public static final double GOBILDA_TPR = 537.6;
    public static final double WHEEL_DIAMETER = 4.0; // Inches
    public static final double GEAR_RATIO = 10.0 / 15.0;

    // Declare variables

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;
    public Servo turretHood;
    public float drivetrainVelocityScale;

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

        setAllToDefaultDirection();
    }

    private void setAllToDefaultDirection() {
        // Set drivetrain motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        // Set mechanism motor directions
        intake.setDirection(DcMotor.Direction.REVERSE);
        transfer.setDirection(DcMotor.Direction.FORWARD);
        turretFlywheel.setDirection(DcMotor.Direction.REVERSE);
        turretYaw.setDirection(DcMotor.Direction.REVERSE);

        // Set servo direction
        turretHood.setDirection(Servo.Direction.FORWARD);
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
}
