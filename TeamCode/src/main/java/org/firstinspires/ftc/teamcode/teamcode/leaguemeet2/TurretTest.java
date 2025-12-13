package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.LimelightController;

import java.util.List;


// Large Ring Gear: 350T
// Small Pulley: 20T
// Motor: 28 Ticks/Revolution
// Gearbox: 3:1
// 1,470 Ticks/Turret Rotation

/*
Strategy: Start turret looking forward (encoder position 0)
Limits: (-90 degrees, +90 degrees) or (-735 ticks, +735 ticks)
 */
@TeleOp(name="Turret Test", group="League Meet 2")
public class TurretTest extends LinearOpMode{
    public static final double TICKS_PER_RADIAN = 233.96; // Ticks from motor per turret radian
    public static final int TICKS_PER_REVOLUTION = 1470; // Ticks from motor per rotation
    public static final double TICKS_PER_DEGREE = 4.083;
    public static final int TURRET_TICK_LIMIT = 350;
    public static final double FLYWHEEL_TPR = 28.0; // TPS


    public DcMotorEx intake, transfer, turretFlywheel, turretYaw;
    public Servo turretHood;

    public int turretPosition = 0;
    public int targetPosition = 0;
    public double targetSpeed = 1.0; // Target flywheel speed RPS
    public double hoodPosition = 0.5;
    public double distance = 0.0;
    // Button states
    public boolean a_btn_state = false;
    public boolean b_btn_state = false;
    public boolean x_btn_state = false;
    public boolean y_btn_state = false;
    public boolean r_bumper_state = false;

    public boolean turret_state = false;
    public Limelight3A limelight;

    public void runOpMode() {
        // Map mechanism motors
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        transfer = hardwareMap.get(DcMotorEx.class, "transfer");
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "turretFlywheel");
        turretYaw = hardwareMap.get(DcMotorEx.class, "turretYaw");

        // Map turret hood servo
        turretHood = hardwareMap.get(Servo.class, "turretHood");

        // Reset flywheel yaw motor
        turretYaw.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretYaw.setTargetPosition(0);
        turretYaw.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //turretYaw.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretYaw.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretYaw.setPower(0.3);


        // Set mechanism motor directions
        intake.setDirection(DcMotorEx.Direction.FORWARD);
        transfer.setDirection(DcMotorEx.Direction.FORWARD);
        turretFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        turretYaw.setDirection(DcMotorEx.Direction.REVERSE);

        // Set mechanisms to PID
        turretFlywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Set servo position
        turretHood.setPosition(hoodPosition);



        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
        limelight.pipelineSwitch(0); // Switch to pipeline number 0

        waitForStart();
        while(opModeIsActive()) {
            // limelight code
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    int id = fiducial.getFiducialId(); // The ID number of the fiducial
                    if(id == 24) {
                        double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                        updateTurretTarget(x);

                        double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                        double targetArea = fiducial.getTargetArea();
                        distance = getTargetDist(targetArea);
                        telemetry.addData("Fiducial " + id, "is " + distance + " meters away");
                        telemetry.addData("Delta Angle X: ", x);
                        telemetry.addData("Delta Angle Y: ", y);
                        telemetry.addData("Target Area: ", targetArea);
                    }
                }
            } else {
                telemetry.addData("Limelight", "No Targets");
            }

            // Motion testing:
            if (gamepad1.a != a_btn_state && !a_btn_state) {
                targetPosition += 100;
                //hoodPosition += 0.05;
            }
            if (gamepad1.b != b_btn_state && !b_btn_state) {
                targetPosition -= 100;
                //hoodPosition -= 0.05;
            }
            if (gamepad1.x != x_btn_state && !x_btn_state) {
                targetSpeed += 1.0;
            }
            if (gamepad1.y != y_btn_state && !y_btn_state) {
                targetSpeed -= 1.0;
            }
            if (gamepad1.right_trigger >= 0.5) {
                intake.setPower(0.6);
                transfer.setPower(0.4);
            } else {
                intake.setPower(0.0);
                transfer.setPower(0.0);
            }


            // Run turret flywheel at targetSpeed
            if (gamepad1.right_bumper != r_bumper_state && !r_bumper_state) {turret_state = !turret_state;}
            if (turret_state) {turretFlywheel.setVelocity(targetSpeed*28.0);} else {turretFlywheel.setVelocity(0.0);}

            targetSpeed = 0.11 * distance + 34.0;
            hoodPosition = Math.max(Math.min(0.55 - (0.00153 * distance) + (0.00000301 * Math.pow(distance, 2)), 0.5), 0.3);

            // Set hood target angle
            turretHood.setPosition(hoodPosition);

            // Update turret velocity for custom PID
            //updateTurretVelocity();

            // Run turret to targetPosition
            turretYaw.setTargetPosition(targetPosition);

            // Update button states:
            a_btn_state = gamepad1.a;
            b_btn_state = gamepad1.b;
            x_btn_state = gamepad1.x;
            y_btn_state = gamepad1.y;
            r_bumper_state = gamepad1.right_bumper;

            // Get current turret position
            //turretPosition = turretYaw.getCurrentPosition();
            // Update telemetry information for encoder values
            telemetry.addData("Turret Position (Ticks): ", turretYaw.getCurrentPosition());
            telemetry.addData("Turret Position (Degrees): ", (double) turretPosition / (double) TICKS_PER_REVOLUTION);
            telemetry.addData("Target Position (Ticks): ", targetPosition);
            telemetry.addData("Target Speed (RPS: ", targetSpeed);
            telemetry.addData("Current Speed (RPS): ", turretFlywheel.getVelocity() / 28.0);
            telemetry.addData("Hood Position (0.0-1.0: ", hoodPosition);

            telemetry.update();
        }
    }

    public void updateTurretTarget(double deltaAngle) {
        if (Math.abs(deltaAngle) < 3.0) {return;}
        int deltaTicks = (int) (deltaAngle * TICKS_PER_DEGREE);
        telemetry.addData("Delta Ticks: ", deltaTicks);
        int currentPosition = turretYaw.getCurrentPosition();
        int tempPos = currentPosition + deltaTicks;
        targetPosition = Math.max(Math.min(tempPos, TURRET_TICK_LIMIT), -TURRET_TICK_LIMIT);
    }


    public double getTargetDist(double targetArea) {
        double scale = 14.76;
        return scale/Math.sqrt(targetArea);
    }
    public static double toRPS(double tps) { return tps / FLYWHEEL_TPR; }

}
