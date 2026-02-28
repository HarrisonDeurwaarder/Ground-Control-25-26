package org.firstinspires.ftc.teamcode.teamcode.state.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.FuturePose;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;

@Config
@TeleOp(name = "TeleOp", group = "State")
public class TeleOpPackage extends SelectableOpMode {
    public static Follower follower;
    public static double SLOW_MODE_MULTIPLIER = 0.2;
    public static boolean using2Drivers = true;
    public static boolean invertControls = true;
    public static boolean autoShooting = false;
    public static boolean debugLift = false;
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


class DebuggerTeleOp extends OpMode {

    protected Timer opmodeTimer;
    protected Follower follower;
    protected HardwareController hardwareController;
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;

    // Poses
    protected Pose startingPose     = new Pose(0.0, 0.0, Math.toRadians(90.0));
    protected Pose goalPose         = new Pose(60.0, 60.0);
    protected Pose recalibratedPose = new Pose(-64.0, -63.0, Math.toRadians(90.0));

    // Boolean flags
    protected boolean isRobotCentric = true;
    protected boolean slowMode = false;

    // Macro variables

    protected int scoringMacroState = 0;
    protected int parkingMacroState = 0;
    protected double dpadDownHoldTime = 0.0;
    public static boolean scoringMacroEnabled = false;
    public static boolean parkingMacroEnabled = false;

    protected Pose parkingPose = new Pose(0.0, 0.0, -90.0);
    protected FuturePose currentRobotPose, scoringPose;
    protected Path scorePath, parkPath;

    // Constants


    public static double clutchPower = 0.3;
    public static double liftPower = 1.0;

    public boolean clutchEngaged = false;
    public int liftPositionLeft = 0;
    public int liftPositionRight = 0;
    public static int liftIncrement = 80
    public static int tolerance = 10;
    protected boolean liftMode = false;
    protected boolean liftingStarted = false;
    protected double timeSinceLiftingStarted = 0.0;

    // Lift Constants

    @Override
    public final void init() {

        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);
        HardwareController.enableVirtualRobotPose = false;
        HardwareController.enableVirtualGoalPose = false;
        HardwareController.enableAutoAiming = true;
        HardwareController.enableFlywheel = true;

        // Configure follower
        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        //loadMacroPaths();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        follower.startTeleOpDrive(true);
    }

    @Override
    public final void init_loop() {
        // Display controls on gamepad
        displayControls();
    }

    @Override
    public final void loop() {
        // Display controls on gamepad
        displayControls();
        follower.update();
        // Lifting block
        if (!liftMode) {
            setTeleOpDrive();
        }

        // Auto shoot
        if (TeleOpPackage.autoShooting && HardwareController.enableAutoAiming && HardwareController.enableFlywheel) {
            // Feed if in zone
            if (hardwareController.inShootingZone(follower)) {
                hardwareController.gate.setPosition(HardwareController.GATE_OPEN_ANGLE);
            }
            // Stop feeding if not in zone (and manual trigger is not active)
            else {
                hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
            }
        }

        // C0nstantly update servo pos
        if (!liftMode) {
            hardwareController.clutchLeft.setPosition(HardwareController.LEFT_CLUTCH_DRIVE_ANGLE);
            hardwareController.clutchRight.setPosition(HardwareController.RIGHT_CLUTCH_DRIVE_ANGLE);
        }

        // Update controls
        if (!TeleOpPackage.using2Drivers) {
            updateControls1Driver();
        } else {
            updateControls2Drivers();
        }

        // Perform turret updates
        hardwareController.updateTurret(follower, goalPose);
        // Update dashboard telemetry
        updateTelemetry();
    }

    @Override
    public final void stop() {
        // Disable all motors
        hardwareController.gate.setPosition(0.5);
        hardwareController.intake.setPower(0.0);
        follower.breakFollowing();

        // Reset turret
        hardwareController.flywheelA.setPower(0.0);
        hardwareController.flywheelB.setPower(0.0);
        hardwareController.updateTurretTarget(0.0);
    }

    protected double[] getDefaultDriveControls() {
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
        };
        return controls;
    }

    protected final void setTeleOpDrive() {
        double[] controls = getDefaultDriveControls();
        // Write over default controls if robot-centric is enabled
        if (isRobotCentric) {
            controls[0] = -gamepad1.left_stick_y;
            controls[1] = -gamepad1.left_stick_x;
            controls[2] = -gamepad1.right_stick_x;
        }
        // Scale all controls if precision mode is enabled
        if (slowMode) {
            controls[0] *= TeleOpPackage.SLOW_MODE_MULTIPLIER;
            controls[1] *= TeleOpPackage.SLOW_MODE_MULTIPLIER;
            controls[2] *= TeleOpPackage.SLOW_MODE_MULTIPLIER;
        }

        // Set drive
        follower.setTeleOpDrive(
                controls[0],
                controls[1],
                controls[2],
                isRobotCentric
        );
    }

    /* MACROS */

    protected final void loadMacroPaths() {
        currentRobotPose = () -> follower.getPose();
        scoringPose = () -> hardwareController.getNearestShootingPose(follower);

        // Shooting path
        scorePath = new Path(new BezierLine(currentRobotPose, scoringPose));
        scorePath.setTangentHeadingInterpolation();
        // Parking path
        parkPath = new Path(new BezierLine(currentRobotPose, parkingPose));
        parkPath.setTangentHeadingInterpolation();
    }

    protected final void scoringMacro() {
        switch (scoringMacroState) {
            // Break other path following
            case 0:
                follower.breakFollowing();
                scoringMacroState = 1;
                break;
            // Follow score path
            case 1:
                follower.followPath(scorePath, true);
                scoringMacroState = 2;
                break;
            // Set slow mode
            case 2:
                if (!follower.isBusy()) {
                    slowMode = true;
                    scoringMacroState = -1;
                }
                break;
        }
    }

    protected final void parkingMacro() {
        switch (parkingMacroState) {
            // Break other path following
            case 0:
                follower.breakFollowing();
                parkingMacroState = 1;
                break;
            // Follow park path
            case 1:
                follower.followPath(scorePath, true);
                parkingMacroState = 2;
                break;
            // Feed
            case 2:
                if (!follower.isBusy()) {
                    slowMode = true;
                    parkingMacroState = -1;
                }
                break;
        }
    }

    /* CONTROL CONFIGS */

    protected final void updateControls1Driver() {
        // Recalibrate pose
        if (gamepad1.dpadUpWasPressed()) {
            // Override follower pose
            follower.setPose(recalibratedPose.copy());
        }

        // Toggle slow mode
        if (gamepad1.aWasPressed()) slowMode = !slowMode;
        // Toggle robot-centered
        if (gamepad1.bWasPressed()) isRobotCentric = !isRobotCentric;
        // Start lifting
        if (gamepad2.left_bumper && gamepad2.right_bumper && (opmodeTimer.getElapsedTimeSeconds() >= 100.0 || TeleOpPackage.debugLift)) {
            liftMode = true;
            hardwareController.clutchLeft.setPosition(hardwareController.LEFT_CLUTCH_LIFT_ANGLE);
            hardwareController.clutchRight.setPosition(hardwareController.RIGHT_CLUTCH_LIFT_ANGLE);        }

        // Lifting loop
        if (liftMode) {
            if (!liftingStarted) {
                if (gamepad2.left_trigger > 0.05)
                {
                    hardwareController.leftFront.setPower(-clutchPower);
                    hardwareController.rightFront.setPower(-clutchPower);
                    hardwareController.leftBack.setPower(clutchPower/2.0);
                    hardwareController.rightBack.setPower(clutchPower/2.0);
                }
                else {
                    hardwareController.leftFront.setPower(0.0);
                    hardwareController.rightFront.setPower(0.0);
                    hardwareController.leftBack.setPower(0.0);
                    hardwareController.rightBack.setPower(0.0);
                }
            }
            else {
                hardwareController.rightFront.setPower(liftPower);
                hardwareController.leftFront.setPower(liftPower);
                if (Math.abs(hardwareController.rightFront.getCurrentPosition() - liftPositionRight) < tolerance
                        && Math.abs(hardwareController.leftFront.getCurrentPosition() - liftPositionLeft) < tolerance) {
                    if (gamepad2.left_bumper){
                        liftPositionLeft -= liftIncrement;
                        hardwareController.leftFront.setTargetPosition(liftPositionLeft);
                    }
                    if (gamepad2.right_bumper) {
                        liftPositionRight -= liftIncrement;
                        hardwareController.rightFront.setTargetPosition(liftPositionRight);
                    }
                }
            }
            if (gamepad2.xWasPressed()) {
                liftingStarted = true;
                hardwareController.activateLift();
            }

        }
        // Toggle auto-aiming
        if (gamepad1.xWasPressed()) HardwareController.enableAutoAiming = !HardwareController.enableAutoAiming;
        // Toggle flywheel
        if (gamepad1.yWasPressed()) HardwareController.enableFlywheel = !HardwareController.enableFlywheel;

        // Parking macro
        if (gamepad1.dpad_down) parkingMacro();

        // Switch gate to closed only if robot is not feeding
        if ((TeleOpPackage.invertControls ? gamepad2.left_trigger : gamepad2.right_trigger) < 0.05 && !(TeleOpPackage.autoShooting && hardwareController.inShootingZone(follower))) hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);

        // FEEDING CONDITIONAL
        // When trigger is held and flywheel velocity is acceptable, feed
        if ((TeleOpPackage.invertControls ? gamepad1.left_trigger : gamepad1.right_trigger) >= 0.05 || (TeleOpPackage.autoShooting && hardwareController.inShootingZone(follower))) {
            // Switch gate to open
            hardwareController.gate.setPosition(HardwareController.GATE_OPEN_ANGLE);
            // Switch intake mode to [intake] if needed
            if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            // Then feed and intake
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // INTAKE CONDITIONAL
        // When trigger is held, intake
        else if ((TeleOpPackage.invertControls ? gamepad1.right_trigger : gamepad1.left_trigger) >= 0.05) {
            // Switch intake mode to reverse if needed
            if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            // Then power intake and gate
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // OUTTAKE CONDITIONAL
        // When trigger is held, intake
        else if (TeleOpPackage.invertControls ? gamepad1.right_bumper : gamepad1.left_bumper) {
            // Switch intake mode to reverse if needed
            if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            // Then power intake and gate
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // Else don't power either motor
        else {
            hardwareController.intake.setPower(0.0);
        }

        /*// Scoring macro
        if (gamepad1.dpad_left) {
            // Initial check to reset counter
            if (gamepad1.dpadLeftWasPressed()) scoringMacroState = 0;
            // Execute if not in scoring zone
            if (!hardwareController.inShootingZone()) {
                scoringMacro();
            }
            // Else enable feeding
            else {
                hardwareController.gate.setPosition(HardwareController.GATE_OPEN_ANGLE);
            }
        } else {
            hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
        }
        // Parking macro
        if (gamepad1.dpad_right && !gamepad1.dpad_left) {
            // Initial check to reset counter
            if (gamepad1.dpadRightWasPressed()) parkingMacroState = 0;
            // Execute
            parkingMacro();
        }*/
    }

    protected final void updateControls2Drivers() {

        /* DRIVER #1 */

        // Toggle slow mode
        if (gamepad1.aWasPressed()) slowMode = !slowMode;
        // Toggle robot-centered
        if (gamepad1.bWasPressed()) isRobotCentric = !isRobotCentric;

        /* DRIVER #2 */

        // Recalibrate pose
        if (gamepad2.dpadUpWasPressed()) {
            // Override follower pose
            follower.setPose(recalibratedPose.copy());
        }
        // Start lifting
        if (gamepad2.left_bumper && gamepad2.right_bumper && (opmodeTimer.getElapsedTimeSeconds() >= 100.0 || TeleOpPackage.debugLift)) {
            liftMode = true;
            // Only reset time at start
            if (timeSinceLiftingStarted == 0.0) {
                timeSinceLiftingStarted = opmodeTimer.getElapsedTimeSeconds();
            }
            // Commence lifting
            hardwareController.startLift(timeSinceLiftingStarted);
        }
        // Toggle auto-aiming
        if (gamepad2.aWasPressed()) HardwareController.enableAutoAiming = !HardwareController.enableAutoAiming;
        // Toggle flywheel
        if (gamepad2.bWasPressed()) HardwareController.enableFlywheel = !HardwareController.enableFlywheel;
        // Toggle autoshooting
        // if (gamepad2.xWasPressed()) TeleOpPackage.autoShooting = !TeleOpPackage.autoShooting;
        // Manually override turret rotation
        if (gamepad2.dpad_right) hardwareController.manualRotationOverride--;
        if (gamepad2.dpad_left) hardwareController.manualRotationOverride++;

        // Switch gate to closed only if robot is not feeding
        if ((TeleOpPackage.invertControls ? gamepad2.left_trigger : gamepad2.right_trigger) < 0.05 && !(TeleOpPackage.autoShooting && hardwareController.inShootingZone(follower))) hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);

        // FEEDING CONDITIONAL
        // When trigger is held and flywheel velocity is acceptable, feed
        if ((TeleOpPackage.invertControls ? gamepad2.left_trigger : gamepad2.right_trigger) >= 0.05 || (TeleOpPackage.autoShooting && hardwareController.inShootingZone(follower))) {
            // Switch gate to open
            hardwareController.gate.setPosition(HardwareController.GATE_OPEN_ANGLE);
            // Switch intake mode to [intake] if needed
            if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            // Then feed and intake
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // INTAKE CONDITIONAL
        // When trigger is held, intake
        else if ((TeleOpPackage.invertControls ? gamepad2.right_trigger : gamepad2.left_trigger) >= 0.05 || gamepad1.right_trigger >= 0.05) {
            // Switch intake mode to reverse if needed
            if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
            }
            // Then power intake and gate
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // OUTTAKE CONDITIONAL
        // When trigger is held, intake
        else if (TeleOpPackage.invertControls ? gamepad2.right_bumper : gamepad2.left_bumper) {
            // Switch intake mode to reverse if needed
            if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
            }
            // Then power intake and gate
            hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        }

        // Else don't power either motor
        else {
            hardwareController.intake.setPower(0.0);
        }
    }

    /* TELEMETRY */

    protected final void updateTelemetry() {
        // Debug telemetry (On panels)
        packet.put("Position (In)", follower.getPose());
        packet.put("Velocity (In/Sec)", follower.getVelocity());
        packet.put("Flywheel Velocity (RPS)", hardwareController.flywheelA.getVelocity() / (HardwareController.FLYWHEEL_TICKS_PER_DEGREE * 360));

        packet.put("Virtual Goal Offset", goalPose.minus(new Pose(
                follower.getPose().getX() + HardwareController.FEEDING_LATENCY * follower.getVelocity().getXComponent() + (Math.pow(HardwareController.FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getXComponent(),
                follower.getPose().getY() + HardwareController.FEEDING_LATENCY * follower.getVelocity().getYComponent() + (Math.pow(HardwareController.FEEDING_LATENCY, 2) / 2) * follower.getAcceleration().getYComponent(),
                follower.getHeading() //+ FEEDING_LATENCY * follower.getAngularVelocity()
        )));
        packet.put("Virtual Robot Offset", follower.getPose().minus(new Pose(
                goalPose.getX() - hardwareController.computeAirtime(follower.getPose(), goalPose) * (follower.getVelocity().getXComponent() + HardwareController.FEEDING_LATENCY * follower.getAcceleration().getXComponent()),
                goalPose.getY() - hardwareController.computeAirtime(follower.getPose(), goalPose) * (follower.getVelocity().getYComponent() + HardwareController.FEEDING_LATENCY * follower.getAcceleration().getYComponent())
        )));

        packet.put("In Shooting Zone", hardwareController.inShootingZone(follower));
        packet.put("Shooting Pose", hardwareController.getNearestShootingPose(follower));
        packet.put("Upper Zone", follower.getPose().getY() + HardwareController.SHOOTING_TOLERANCE >= Math.abs(follower.getPose().getX()));
        packet.put("Lower Zone", follower.getPose().getY() - HardwareController.SHOOTING_TOLERANCE + 48.0 <= -Math.abs(follower.getPose().getX()));

        packet.put("Turret Rotation Current", hardwareController.turretRotation.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Flywheel1 Current", hardwareController.flywheelA.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Flywheel2 Current", hardwareController.flywheelB.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Intake Current", hardwareController.intake.getCurrent(CurrentUnit.MILLIAMPS));

        packet.put("Left Clutch Position", hardwareController.clutchLeft.getPosition());
        packet.put("Right Clutch Position", hardwareController.clutchRight.getPosition());

        packet.put("Turret Angle", hardwareController.turretAngle);
        packet.put("Turret Ticks", hardwareController.turretTicks);
        packet.put("Power", HardwareController.turretRotationPID.compute(hardwareController.turretTicks, hardwareController.turretRotation.getCurrentPosition()));

        packet.put("Distance", hardwareController.distance);

        packet.put("Left Motor Current", hardwareController.leftFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Right Motor Current", hardwareController.rightFront.getCurrent(CurrentUnit.MILLIAMPS));
        packet.put("Left Motor Position", hardwareController.leftFront.getCurrentPosition());
        packet.put("Right Motor Position", hardwareController.rightFront.getCurrentPosition());
        packet.put("Lift Started", liftingStarted);
        packet.put("Lift Mode", liftMode);

        dashboard.sendTelemetryPacket(packet);
    }

    protected final void displayControls() {
        // Controls (On driver hub telemetry)
        if (TeleOpPackage.using2Drivers) {
            // Driver 1
            telemetry.addLine("DRIVER 1\n");

            telemetry.addLine("RT - Intake");
            telemetry.addLine("RB - Outtake\n");

            telemetry.addLine("A - Precision Mode");
            telemetry.addLine("B - Robot Oriented\n");

            // Driver 2
            telemetry.addLine("DRIVER 2\n");

            telemetry.addLine("LT - Feed\n");

            telemetry.addLine("A - Auto Aim Turret");
            telemetry.addLine("B - Enable Flywheel\n");

            telemetry.addLine("DPad ↑ - Recalibrate Odometry");
            telemetry.addLine("DPad ↓ - Lifting Sequence");

        } else {
            telemetry.addLine("A - Precision Mode");
            telemetry.addLine("B - Robot Oriented");
            telemetry.addLine("X - Auto Aim Turret");
            telemetry.addLine("Y - Enable Flywheel\n");

            telemetry.addLine("RT - Intake");
            telemetry.addLine("RB - Outtake");
            telemetry.addLine("LT - Feed\n");

            telemetry.addLine("DPad ↑ - Recalibrate Odometry");
            telemetry.addLine("DPad ↓ - Lifting Sequence");
        }
        // Send telemetry
        telemetry.update();
    }
}


class RedNearTeleOp extends DebuggerTeleOp {
    RedNearTeleOp() {
        super();
        // Reassign poses
        this.startingPose     = new Pose(45.9, 0.0, Math.toRadians(90));
        this.goalPose         = new Pose(60.0, 64.0);
        this.recalibratedPose = new Pose(-64.0, -63.0, Math.toRadians(90.0));
    }

    @Override
    protected double[] getDefaultDriveControls() {
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
        };
        return controls;
    }
}


class RedFarTeleOp extends DebuggerTeleOp {
    RedFarTeleOp() {
        super();
        // Reassign poses
        this.startingPose     = new Pose(36.0, -63.0, Math.toRadians(90.0));
        this.goalPose         = new Pose(60.0, 60.0);
        this.recalibratedPose = new Pose(-64.0, -63.0, Math.toRadians(90.0));
    }

    @Override
    protected double[] getDefaultDriveControls() {
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                -gamepad1.right_stick_x,
        };
        return controls;
    }
}


class BlueNearTeleOp extends DebuggerTeleOp {
    BlueNearTeleOp() {
        super();
        // Reassign poses
        this.startingPose     = new Pose(-47.8, 0.0, Math.toRadians(90));
        this.goalPose         = new Pose(-60.0, 60.0);
        this.recalibratedPose = new Pose(64.0, -63.0, Math.toRadians(90.0));
    }

    @Override
    protected double[] getDefaultDriveControls() {
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x,
        };
        return controls;
    }
}


class BlueFarTeleOp extends DebuggerTeleOp {
    BlueFarTeleOp() {
        super();
        // Reassign poses
        this.startingPose     = new Pose(-36.0, -63.0, Math.toRadians(90.0));
        this.goalPose         = new Pose(-60.0, 60.0);
        this.recalibratedPose = new Pose(-64.0, -63.0, Math.toRadians(90.0));
    }

    @Override
    protected double[] getDefaultDriveControls() {
        double[] controls = {
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                gamepad1.right_stick_x,
        };
        return controls;
    }
}