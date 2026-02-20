package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Motor Test", group="Test")
public class MotorTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx flywheelA;
    private DcMotorEx flywheelB;
    private DcMotorEx turretRotation;
    private DcMotorEx intake;

    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;

    private Servo gate;
    private Servo leftClutch;
    private Servo rightClutch;

    private double motor_power = 0.5;
    private int motor_state = 1;

    public static double Kp = 0.5;
    public static double Kd = 0.0003;
    public static double Ks = 0.2;

    public double targetSpeed = 20.0;
    public double lastRecordedError = 0.0;
    public double lastRecordedTime = 0.0;
    private boolean flywheel_active = false;

    @Override
    public void runOpMode() {
        flywheelA = hardwareMap.get(DcMotorEx.class, "flywheelA");
        flywheelB = hardwareMap.get(DcMotorEx.class, "flywheelB");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        turretRotation = hardwareMap.get(DcMotorEx.class, "turretRotation");

        gate = hardwareMap.get(Servo.class, "gate");

        flywheelA.setDirection(DcMotorEx.Direction.FORWARD);
        flywheelB.setDirection(DcMotorEx.Direction.FORWARD);
        leftFront.setDirection(DcMotorEx.Direction.FORWARD);
        leftBack.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);
        rightBack.setDirection(DcMotorEx.Direction.FORWARD);
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        turretRotation.setDirection(DcMotorEx.Direction.FORWARD);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.bWasPressed()) {
                triggerMotor(motor_state);
            }
            if (gamepad1.aWasPressed()) {
                motor_state = (motor_state + 1) % 7;
            }
            if (gamepad1.xWasPressed()) {motor_power = Math.min(motor_power + 0.1, 1.0);}
            if (gamepad1.yWasPressed()) {motor_power = Math.max(motor_power - 0.1, -1.0);}

            if (gamepad1.rightBumperWasPressed()){flywheel_active = !flywheel_active;}
            if (flywheel_active) {PDController(runtime.time() - lastRecordedTime);}

            lastRecordedTime = runtime.time();

            if (gamepad1.dpadDownWasPressed()) {targetSpeed -= 1.0;}
            if (gamepad1.dpadUpWasPressed()) {targetSpeed += 1.0;}

            telemetry.addData("Motor State: ", motor_state);
            telemetry.addData("Motor Power: ", motor_power);
            telemetry.addData("Flywheel Active: ", flywheel_active);
            telemetry.addData("Flywheel velocity (RPS):", flywheelA.getVelocity()/28.0);
            telemetry.addData("Flywheel target (RPS):", targetSpeed);
            telemetry.update();

        }

    }
    public void triggerMotor (int state) {
        switch (motor_state) {
            case 0:
                flywheelA.setPower((flywheelA.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 1:
                flywheelB.setPower((flywheelB.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 2:
                intake.setPower((intake.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 3:
                //turretRotation.setPower((turretRotation.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 4:
                leftFront.setPower((leftFront.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 5:
                rightFront.setPower((rightFront.getPower() > 0.0) ? 0 : motor_power);
                break;
            case 6:
                break;
        }
        return;
    }
    public void PDController(double deltaTime) {
        double proportional = targetSpeed - (flywheelA.getVelocity() / 28.0);
        double derivative   = (proportional - lastRecordedError) / deltaTime;
        double constant     = Math.signum(proportional);
        // Compute power
        double power = Kp * proportional + Kd * derivative + Ks * constant;
        // Clip power
        power = Math.max(Math.min(power, 1.0), -1.0);
        flywheelA.setPower(power);
        flywheelB.setPower(power);
        // Save previous
        lastRecordedError = proportional;
    }

    // Hood Angle: 0.00438x + 0.0457
    // Flywheel Speed (RPS): 0.176x + 33.9
}

