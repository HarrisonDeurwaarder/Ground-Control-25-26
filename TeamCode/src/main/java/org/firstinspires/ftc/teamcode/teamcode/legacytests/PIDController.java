package org.firstinspires.ftc.teamcode.teamcode.legacytests;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="PID Tester", group="Tests")

public class PIDController extends LinearOpMode {

    // Turret velocity PID variables
    public double turretVelocityError = 0.0;
    public double turretPreviousError = 0.0;
    public double turretTotalError = 0.0;
    public double targetVelocity = 0.0;
    public double Kp = 0.025;
    public double Kd = 0.0;
    public double Ki = 0.0;
    public double FLYWHEEL_SPEED = 45.0;
    public double FLYWHEEL_TPR = 28.0;
    private boolean flywheelState = false;
    private ElapsedTime runtime = new ElapsedTime();
    private double prevTime = 0.0;
    private Limelight3A limelight;
    public DcMotorEx turretFlywheel;


    @Override
    public void runOpMode() {
        turretFlywheel = hardwareMap.get(DcMotorEx.class, "turretFlywheel");
        turretFlywheel.setDirection(DcMotorEx.Direction.REVERSE);
        turretFlywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turretFlywheel.setPower(0.0);

        waitForStart();
        runtime.reset();

        while(opModeIsActive()) {
            if (gamepad1.rightBumperWasPressed()) {
                flywheelState = !flywheelState;
                setFlywheelVelocity((flywheelState) ? FLYWHEEL_SPEED : 0.0);
            }
            if (gamepad1.aWasPressed()) {Kp += 0.005;}
            if (gamepad1.bWasPressed()) {Kp -= 0.005;}
            if (gamepad1.xWasPressed()) {Ki += 0.005;}
            if (gamepad1.yWasPressed()) {Ki -= 0.005;}

            double deltaTime = runtime.time() - prevTime;
            prevTime = runtime.time();
            turretFlywheel.setPower((flywheelState) ? PID_Controller(deltaTime) : 0.0);
            telemetry.addData("Flywheel Velocity: ", turretFlywheel.getVelocity() / FLYWHEEL_TPR);
            telemetry.addData("Target Velocity: ", targetVelocity);
            telemetry.addData("PID Controller Output: ", PID_Controller(deltaTime));
            telemetry.addData("Kp: ", Kp);
            telemetry.addData("Kd: ", Kd);
            telemetry.addData("Ki: ", Ki);
            telemetry.update();
        }
    }

    /**
     * PID controller for turret velocity
     *
     * @param deltaTime time since last run cycle
     * @return power output power for flywheel motor
     */
    public double PID_Controller(double deltaTime) {
        turretVelocityError = targetVelocity - (turretFlywheel.getVelocity() / FLYWHEEL_TPR); // P-value
        double turretDeltaError = (turretVelocityError - turretPreviousError) / deltaTime; // D-value
        turretTotalError += (turretVelocityError * deltaTime); // I-value

        turretPreviousError = turretVelocityError;

        double power = (Kp * turretVelocityError) + (Kd * turretDeltaError) + (Ki * turretTotalError);
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
