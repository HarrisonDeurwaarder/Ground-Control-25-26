package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class MotorDriver {
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

    public DcMotorEx leftFront, leftBack, rightFront, rightBack;
    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;

    public MotorDriver(HardwareMap hardwareMap) {
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
    }
}
