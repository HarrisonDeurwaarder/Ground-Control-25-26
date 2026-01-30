package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.PoseHistory;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.leaguetournament.HardwareController;

import java.util.ArrayList;

@Configurable
@TeleOp(name = "TeleOp", group = "League Tournament")
public class TeleOpPackage extends SelectableOpMode {
    public static Follower follower;
    public TeleOpPackage() {
        super("Select a TeleOp", s -> {
            s.add("Red Near", RedNearTeleOp::new);
            s.add("Red Far", RedFarTeleOp::new);
            s.add("Blue Near", BlueNearTeleOp::new);
            s.add("Blue Far", BlueFarTeleOp::new);
            s.add("Debugger", DebuggerTeleOp::new);
        });
    }
}


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Debugger", group="League Tournament")
class DebuggerTeleOp extends LinearOpMode {

    protected Timer opmodeTimer;
    protected Follower follower;
    protected HardwareController hardwareController;
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // Poses
    public static Pose startingPose = new Pose(0.0, 0.0, Math.toRadians(90.0));
    public static Pose goalPose     = new Pose(60.0, 60.0);

    // Boolean flags
    protected boolean isRobotCentric = false;
    protected boolean slowMode = false;

    // Constants
    protected static double SLOW_MODE_MULTIPLIER = 0.2;

    @Override
    public final void runOpMode() {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);

        // Configure follower
        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        follower.startTeleOpDrive(true);

        // Display controls on gamepad
        displayControls();

        waitForStart();

        /* ###############################
                        START
           ############################### */

        while (opModeIsActive()) {

            follower.update();
            setTeleOpDrive();

            /* NON-DRIVING CONTROLS */

            // Toggle slow mode
            if (gamepad1.aWasPressed()) slowMode = !slowMode;

            // Toggle robot-centered
            if (gamepad1.bWasPressed()) isRobotCentric = !isRobotCentric;

            // Toggle auto-aiming
            if (gamepad1.xWasPressed()) hardwareController.enableAutoAiming = !hardwareController.enableAutoAiming;

            // Toggle flywheel
            if (gamepad1.yWasPressed()) hardwareController.enableFlywheel = !hardwareController.enableFlywheel;

            // Switch gate to closed only if robot is not feeding
            if (gamepad1.right_trigger < 0.05) hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);

            // FEEDING CONDITIONAL
            // When trigger is held and flywheel velocity is acceptable, feed
            if (gamepad1.right_trigger >= 0.05) {
                // Switch gate to open
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                // Switch intake mode to [intake] if needed
                if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then feed and intake
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // INTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_trigger >= 0.05) {
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then power intake and gate
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // OUTTAKE CONDITIONAL
            // When trigger is held, intake
            else if (gamepad1.left_bumper) {
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then power intake and gate
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // Else don't power either motor
            else {
                hardwareController.intake.setPower(0.0);
                hardwareController.transfer.setPower(0.0);
            }
            // Perform turret updates
            hardwareController.updateTurret(follower, goalPose, opmodeTimer.getElapsedTimeSeconds());

            updateTelemetry();
        }

        // Brake flywheel on
        hardwareController.turretFlywheel.setVelocity(0.0);
    }

    private void setTeleOpDrive() {
        // Normal driving mode
        if (!slowMode) follower.setTeleOpDrive(
                gamepad1.left_stick_x,
                -gamepad1.left_stick_y,
                -gamepad1.right_stick_x,
                isRobotCentric
        );
            // Precision driving mode
        else follower.setTeleOpDrive(
                gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                isRobotCentric
        );
    }

    private void updateTelemetry() {
        // Debug telemetry (On panels)
        packet.put("Position (In)", follower.getPose());
        packet.put("Velocity (In/Sec)", follower.getVelocity());
        packet.put("Flywheel Velocity (RPS)", hardwareController.turretFlywheel.getVelocity() / HardwareController.FLYWHEEL_TICKS_PER_DEGREE);

        packet.put("Turret Angle", hardwareController.turretAngle);
        packet.put("Turret Ticks", hardwareController.turretTicks);
        packet.put("Tag in Frame", hardwareController.tagDetected);

        packet.put("Target Speed", hardwareController.targetSpeed);

        packet.put("Distance", hardwareController.distance);
        dashboard.sendTelemetryPacket(packet);
    }

    private void displayControls() {
        // Controls (On driver hub telemetry)
        telemetry.addLine("A - Precision Mode");
        telemetry.addLine("B - Movement Center");
        telemetry.addLine("X - Auto Aim Turret");
        telemetry.addLine("Y - Enable Flywheel\n");

        telemetry.addLine("LT - Intake");
        telemetry.addLine("LB - Outtake\n");

        telemetry.addLine("RT - Feed");

        telemetry.update();
    }
}


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red Near", group="League Tournament")
class RedNearTeleOp extends DebuggerTeleOp {
    // Set teleop specific poses
    public static Pose startingPose = new Pose(47.8, 0.0, Math.toRadians(90));
    public static Pose goalPose     = new Pose(60.0, 60.0);
    private void setTeleOpDrive() {
        // Normal driving mode
        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
            // Precision driving mode
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                false
        );
    }
}


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Red Far", group="League Tournament")
class RedFarTeleOp extends DebuggerTeleOp {
    // Set teleop specific poses
    public static Pose startingPose = new Pose(47.8, 0.0, Math.toRadians(90));
    public static Pose goalPose     = new Pose(60.0, 60.0);
    private void setTeleOpDrive() {
        // Normal driving mode
        if (!slowMode) follower.setTeleOpDrive(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
            // Precision driving mode
        else follower.setTeleOpDrive(
                -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                false
        );
    }
}


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Blue Near", group="League Tournament")
class BlueNearTeleOp extends DebuggerTeleOp {
    // Set teleop specific poses
    public static Pose startingPose = new Pose(-47.8, 0.0, Math.toRadians(90));
    public static Pose goalPose     = new Pose(60.0, 60.0);
    private void setTeleOpDrive() {
        // Normal driving mode
        if (!slowMode) follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
            // Precision driving mode
        else follower.setTeleOpDrive(
                gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                false
        );
    }
}


@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Blue Near", group="League Tournament")
class BlueFarTeleOp extends DebuggerTeleOp {
    // Set teleop specific poses
    public static Pose startingPose = new Pose(-47.8, 0.0, Math.toRadians(90));
    public static Pose goalPose     = new Pose(60.0, 60.0);
    private void setTeleOpDrive() {
        // Normal driving mode
        if (!slowMode) follower.setTeleOpDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
                false
        );
            // Precision driving mode
        else follower.setTeleOpDrive(
                gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                false
        );
    }
}