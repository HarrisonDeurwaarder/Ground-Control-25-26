package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class GamepadBindingsCfg {

    // Arbitrary toggle/hold maps
    public Map<Supplier<Boolean>, Consumer<Boolean>> toggleKeybinds, holdKeybinds;

    // References to be set later
    private Gamepad gamepad;
    private MotorDriver motorDriver;
    private MotorDriverPID motorDriverPID;
    public double drivetrainPowerScale;
    public GamepadBindingsCfg(Gamepad gamepad, MotorDriver motorDriver, double baseDrivetrainPower) {
        // Set OpMode-specific references
        this.gamepad = gamepad;
        this.motorDriver = motorDriver;
        this.drivetrainPowerScale = baseDrivetrainPower;
    }

    public GamepadBindingsCfg(Gamepad gamepad, MotorDriverPID motorDriver, double baseDrivetrainPower) {
        // Set OpMode-specific references
        this.gamepad = gamepad;
        this.motorDriverPID = motorDriver;
        this.drivetrainPowerScale = baseDrivetrainPower;
    }

    public void initLouis() {
        // Define the toggle keybinds
        toggleKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Cycle to intake
                () -> gamepad.right_trigger > MotorDriver.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriver.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE),
                // Cycle to outtake (both transport and intake motor)
                () -> gamepad.left_trigger > MotorDriver.TRIGGER_THRESHOLD,
                (Boolean mode) -> {
                    motorDriver.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorDriver.transportMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                },
                // Revert transport to default (launch direction)
                () -> gamepad.left_trigger <= MotorDriver.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriver.transportMotor.setDirection(DcMotorSimple.Direction.FORWARD),
                // Toggle flywheel
                () -> gamepad.left_bumper,
                (Boolean mode) -> motorDriver.flywheelMotor.setVelocity((mode) ? MotorDriver.FLYWHEEL_SPEED * MotorDriver.TPR : 0.0),
                // Toggle precision drive mode
                () -> gamepad.a,
                (Boolean mode) -> drivetrainPowerScale = ((mode) ? MotorDriver.PRECISE_DRIVE_POWER : MotorDriver.MAX_DRIVE_POWER)
        );
        // Define the hold keybinds
        holdKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Power the intake/outtake wheel
                () -> gamepad.right_trigger > MotorDriver.TRIGGER_THRESHOLD || gamepad.left_trigger > MotorDriver.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriver.intakeMotor.setPower((mode) ? MotorDriver.INTAKE_POWER : 0.0),
                // Power the transfer mechanism (launch)
                // If it's launching, then make sure flywheel is up to speed
                () -> (gamepad.right_bumper && (true)) || gamepad.left_trigger > MotorDriver.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriver.transportMotor.setPower((mode) ? MotorDriver.TRANSPORT_POWER : 0.0)
        );
    }

    public void initLouisPID() {
        // Define the toggle keybinds
        toggleKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Cycle to intake
                () -> gamepad.right_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD),
                // Cycle to outtake (both transport and intake)
                () -> gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE),
                // Toggle flywheel
                () -> gamepad.left_bumper,
                (Boolean mode) -> motorDriverPID.flywheelMotor.setVelocity((mode) ? MotorDriverPID.toTPS(MotorDriverPID.FLYWHEEL_RPS) : 0.0),
                // Toggle precision drive mode
                () -> gamepad.a,
                (Boolean mode) -> drivetrainPowerScale = ((mode) ? MotorDriverPID.PRECISE_DRIVE_RPS : MotorDriverPID.MAX_DRIVE_RPS),
                // Toggle
                () -> gamepad.right_bumper,
                (Boolean mode) -> motorDriverPID.transportMotor.setDirection(DcMotorSimple.Direction.REVERSE),
                () -> !gamepad.right_bumper,
                (Boolean mode) -> motorDriverPID.transportMotor.setDirection(DcMotorSimple.Direction.FORWARD)
        );
        // Define the hold keybinds
        holdKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Power the intake/outtake wheel
                () -> gamepad.right_trigger > MotorDriverPID.TRIGGER_THRESHOLD || gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.intakeMotor.setPower((mode) ? MotorDriverPID.INTAKE_POWER : 0.0),
                // Power the transfer mechanism (launch)
                () -> (gamepad.right_bumper && motorDriverPID.flywheelInRange()) || gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.transportMotor.setVelocity((mode) ? MotorDriverPID.toTPS(MotorDriverPID.TRANSPORT_RPS) : 0.0)
        );
    }

    public void initHenryPID() {
        // Define the toggle keybinds
        toggleKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Cycle to intake
                () -> gamepad.right_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE),
                // Cycle to outtake (both transport and intake)
                () -> gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> {
                    motorDriverPID.intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                    motorDriverPID.transportMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                },
                // Revert transport to default (launch direction)
                () -> gamepad.left_trigger <= MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.transportMotor.setDirection(DcMotorSimple.Direction.FORWARD),
                // Toggle flywheel
                () -> gamepad.left_bumper,
                (Boolean mode) -> motorDriverPID.flywheelMotor.setVelocity((mode) ? MotorDriverPID.toTPS(MotorDriverPID.FLYWHEEL_RPS) : 0.0),
                // Toggle precision drive mode
                () -> gamepad.a,
                (Boolean mode) -> drivetrainPowerScale = ((mode) ? MotorDriverPID.PRECISE_DRIVE_RPS : MotorDriverPID.MAX_DRIVE_RPS)
        );
        // Define the hold keybinds
        holdKeybinds = Map.<Supplier<Boolean>, Consumer<Boolean>>of(
                // Power the intake/outtake wheel
                () -> gamepad.right_trigger > MotorDriverPID.TRIGGER_THRESHOLD || gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.intakeMotor.setVelocity((mode) ? MotorDriverPID.toTPS(MotorDriverPID.INTAKE_POWER) : 0.0),
                // Power the transfer mechanism (launch)
                () -> gamepad.right_bumper || gamepad.left_trigger > MotorDriverPID.TRIGGER_THRESHOLD,
                (Boolean mode) -> motorDriverPID.transportMotor.setVelocity((mode) ? MotorDriverPID.toTPS(MotorDriverPID.TRANSPORT_RPS) : 0.0)
        );
    }
}
