package org.firstinspires.ftc.teamcode.teamcode.state.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.telemetry.SelectableOpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;


@Config
@Autonomous(name = "Autonomous", group = "State")
public class AutoPackage extends SelectableOpMode {
    public static Follower follower;
    public AutoPackage() {
        super("Select an Auto", s -> {
            s.add("Red Near", RedNearAuto::new);
            s.add("Red Far", RedFarAuto::new);
            s.add("Blue Near", BlueNearAuto::new);
            s.add("Blue Far", BlueFarAuto::new);
        });
    }
}


abstract class DebuggerAuto extends OpMode {
    protected Timer pathTimer, opmodeTimer;
    protected Follower follower;
    protected HardwareController hardwareController;
    protected TelemetryPacket packet;
    protected FtcDashboard dashboard;
    protected int pathState, cycleState = 0;

    public static double FEED_DURATION      = 0.55;
    public static double RC_GATE_DURATION   = 0.0;
    public static double RC_INTAKE_DURATION = 1.0;

    protected Pose goalPose =  new Pose(60.0, 60.0);
    protected Pose startPose = new Pose(40.2, 60.9, Math.toRadians(90.0));
    protected Pose scorePose = new Pose(24.0, 10.8, Math.toRadians(0.0));
    protected boolean overrideAutoShoot = true;

    @Override
    public final void init() {
        // Pedro objects
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        packet = new TelemetryPacket();
        dashboard = FtcDashboard.getInstance();

        follower = ConstantsEpsilon.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        // Hardware controller for mechanism access
        hardwareController = new HardwareController(hardwareMap);
        HardwareController.SHOOTING_TOLERANCE = 3.0;
    }

    @Override
    public final void init_loop() { displayTelemetryMessage(); }

    @Override
    public final void start() {
        pathTimer.resetTimer();
        // Hardware flags
        HardwareController.enableAutoAiming = true;
        HardwareController.enableFlywheel = true;
        HardwareController.enableVirtualGoalPose = true;
        HardwareController.enableVirtualRobotPose = true;
        // Provide constant intake power
        hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
    }

    @Override
    public final void loop() {
        // Loop movements of robot
        follower.update();
        autoPathUpdate();

        // Perform turret updates
        hardwareController.updateTurret(follower, goalPose);
        /*// Auto shoot
        if (hardwareController.inShootingZone(follower) && !overrideAutoShoot) {
            hardwareController.gate.setPosition(HardwareController.GATE_OPEN_ANGLE);
        } else {
            hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
        }*/

        // Log telemetry
        updateTelemetry();

        // Save ending position to blackboard in case of issues
        blackboard.put("Auto Pose", follower.getPose());
    }

    @Override
    public final void stop() {
        // Save ending position to blackboard
        blackboard.put("Auto Pose", follower.getPose());
        // Disable all motors
        hardwareController.gate.setPosition(0.5);
        hardwareController.intake.setPower(0.0);
        follower.breakFollowing();
        // Reset turret
        hardwareController.flywheelA.setPower(0.0);
        hardwareController.flywheelB.setPower(0.0);
        hardwareController.turretRotation.setTargetPosition(0);
    }

    protected abstract void buildPaths();

    protected abstract void autoPathUpdate();

    /**
     * Run preload cycle
     *
     * @param score scoring path
     */
    protected void runPreloadCycle(Path score) {
        switch (pathState) {
            // Go to score position
            case 0:
                follower.followPath(score);
                incrementPathState();
                break;
            // Feed for duration
            case 1:
                // Advance if flywheel is up to speed
                if (!follower.isBusy()) {
                    overrideAutoShoot = false;
                    incrementCycleState();
                }
                break;
        }
    }

    /**
     * Run an artifact set cycle
     *
     * @param grabPickup artifact intake path
     * @param score scoring path
     */
    protected void runArtifactSetCycle(PathChain grabPickup, PathChain score) {
        switch (pathState) {
            // Disable feeder and intake artifacts
            case 0:
                if (pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
                    hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
                    follower.followPath(grabPickup, true);
                    incrementPathState();
                }
                break;
            // Go to score position
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(score, true);
                    incrementPathState();
                }
                break;
            // Get to score position
            case 2:
                if (!follower.isBusy()) incrementCycleState();
                break;
        }
    }

    /**
     * Run a ramp camp cycle
     *
     * @param openGate open gate path
     * @param intake intake path
     * @param score scoring path
     */
    protected void runRCCycle(PathChain openGate, PathChain intake, PathChain score) {
        switch (pathState) {
            // Disable feeder and open gate
            case 0:
                if (pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
                    hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
                    follower.followPath(openGate, true);
                    incrementPathState();
                }
                break;
            // Pause to press gate
            case 1:
                if (!follower.isBusy()) incrementPathState();
                break;
            // Go to intake position
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= RC_GATE_DURATION) {
                    follower.followPath(intake, true);
                    incrementPathState();
                }
                break;
            // Pause to intake
            case 3:
                if (!follower.isBusy()) incrementPathState();
                break;
            // Intake for duration
            case 4:
                if (pathTimer.getElapsedTimeSeconds() >= RC_INTAKE_DURATION) {
                    follower.followPath(score, true);
                    incrementPathState();
                }
                break;
            // Get to score position
            case 5:
                if (!follower.isBusy()) incrementCycleState();
                break;
        }
    }

    /**
     * Run the end auto policy
     */
    protected void runEndAuto(PathChain endAuto) {
        if (pathState == 0 && pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
            hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
            follower.followPath(endAuto, true);

            // Disable all motors
            hardwareController.gate.setPosition(0.5);
            hardwareController.intake.setPower(0.0);

            // Continue only if not shooting
            if (!hardwareController.inShootingZone(follower, true)) {
                // Just in case
                hardwareController.manualRotationOverride = 0;
                HardwareController.enableFlywheel = false;
                HardwareController.enableAutoAiming = false;
                // Disable flywheels
                hardwareController.gate.setPosition(0.5);
                hardwareController.intake.setPower(0.0);
                follower.breakFollowing();
                // Reset turret
                hardwareController.flywheelA.setPower(0.0);
                hardwareController.flywheelB.setPower(0.0);
                hardwareController.turretRotation.setTargetPosition(0);

                incrementCycleState();
            }
        }
    }

    /**
     * Advanced to the next cycle
     */
    protected void incrementCycleState() {
        cycleState++;
        // Cycle state resets always come with path state resets
        pathState = 0;
        pathTimer.resetTimer();
    }

    /**
     * Advanced to the next path state
     */
    protected void incrementPathState() {
        pathState++;
        pathTimer.resetTimer();
    }

    protected void displayTelemetryMessage() {
        telemetry.addLine("An autonomous has been selected. Press start to begin");
    }

    /**
     * Updates panels telemetry
     */
    protected void updateTelemetry() {
        // Write telemetry
        packet.put("Path State", pathState);
        packet.put("Cycle State", cycleState);
        packet.put("Path Timer", pathTimer.getElapsedTime());
        packet.put("Target Pose", follower.getCurrentPath() != null ? follower.getCurrentPath().getPose(1.0) == null : null);

        packet.put("Gate Position", hardwareController.gate.getPosition());

        packet.put("In Shooting Zone?", hardwareController.inShootingZone(follower, true));
        packet.put("Auto Shoot Override?", overrideAutoShoot);

        packet.put("Position (In)", follower.getPose());
        packet.put("Velocity (In/Sec)", follower.getVelocity());
        packet.put("Flywheel Velocity (Rotations/Sec)", hardwareController.flywheelA.getVelocity() / (HardwareController.FLYWHEEL_TICKS_PER_DEGREE * 360));

        packet.put("Flywheel Target Speed (RPS)", hardwareController.targetSpeed);
        packet.put("Turret Target Angle (Degrees)", hardwareController.turretAngle);

        dashboard.sendTelemetryPacket(packet);
    }
}


class RedNearAuto extends DebuggerAuto {
    protected Pose postPickup1Pose =         new Pose(47.5, 16.3, Math.toRadians(0.0));

    protected Pose intermediatePickup2Pose = new Pose(25.9, -6.5, Math.toRadians(0.0));
    protected Pose postPickup2Pose =         new Pose(53.0, -5.5,Math.toRadians(0.0));

    protected Pose intermediatePickup3Pose = new Pose(22.9, -26.8, Math.toRadians(0.0));
    protected Pose postPickup3Pose =         new Pose(53.0, -29.3, Math.toRadians(0.0));

    protected Pose RCIntermediatePose =      new Pose(53.7, -15.9, Math.toRadians(28.5));
    protected Pose RCGatePose =              new Pose(48.6, -0.9, Math.toRadians(0.0));
    protected Pose RCIntakePose =            new Pose(59.3, -15.0, Math.toRadians(43.7));

    protected Pose endAutoPose =             new Pose(14.4, -0.4, Math.toRadians(0.0));
    protected Pose scorePose2 =              new Pose(15.0, 21.0, Math.toRadians(45.0));

    protected Path scorePreload;
    protected PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, openGateRC, intakeRC, scoreRC, endAuto;

    RedNearAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(60.0, 64.0);
        this.startPose = new Pose(37.0, 68.0, Math.toRadians(0.0));
        this.scorePose = new Pose(15.0, 21.0, Math.toRadians(0.0));
    }

    /**
     * Instanciate and build PathChains
     */
    protected void buildPaths() {

        /* ARTIFACT PRELOAD */

        // Shooting position for preloaded artifacts
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* ARTIFACT SET 1 */

        // Curved intake line for artifact set #1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, postPickup1Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Shooting position for artifact set #1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose2.getHeading())
                .build();

        /* ARTIFACT SET 2 */

        // Curved intake line for artifact set #2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup2Pose, postPickup2Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Shooting position for artifact set #2
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(postPickup2Pose, intermediatePickup2Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* ARTIFACT SET 3 */

        // Curved intake line for artifact set #3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup3Pose, postPickup3Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Shooting position for artifact set #3
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        /* RAMP CAMP PROTOCOL */

        // Curved gate open per G418
        openGateRC = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup2Pose, RCGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), RCGatePose.getHeading())
                .build();

        // Ramp camp
        intakeRC = follower.pathBuilder()
                .addPath(new BezierLine(RCGatePose, RCIntakePose))
                .setLinearHeadingInterpolation(RCGatePose.getHeading(), RCIntakePose.getHeading())
                .build();

        // Shooting position for artifact set #1
        scoreRC = follower.pathBuilder()
                .addPath(new BezierCurve(RCIntakePose, intermediatePickup2Pose, scorePose))
                .setLinearHeadingInterpolation(RCIntakePose.getHeading(), scorePose.getHeading())
                .build();

        /* PARKING PROTOCOL */

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endAutoPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endAutoPose.getHeading())
                .build();
    }

    /**
     * Updates the auto paths
     */
    protected void autoPathUpdate(){
        switch (cycleState) {
            // Preload
            case 0:
                runPreloadCycle(scorePreload);
                break;

            // Artifact set 2
            case 1:
                runArtifactSetCycle(grabPickup2, scorePickup2);
                break;

            // RC 1
            case 2:
                runRCCycle(openGateRC, intakeRC, scoreRC);
                break;

            // Artifact set 1
            case 3:
                runArtifactSetCycle(grabPickup1, scorePickup1);
                break;

            // Artifact set 3
            case 4:
                runArtifactSetCycle(grabPickup3, scorePickup3);
                break;

            // End-of-auto parking
            case 5:
                runEndAuto(endAuto);
                break;
        }
    }
}


class RedFarAuto extends DebuggerAuto {
    protected Pose intermediatePickup1Pose = new Pose(10.3, -31.0, Math.toRadians(0.0));
    protected Pose postPickup1Pose =     new Pose(38.0, -29.3, Math.toRadians(0.0));
    protected Pose rcExcessIntakePose1 = new Pose(55.3, -36.2, Math.toRadians(-38.9));
    protected Pose rcExcessIntakePose2 = new Pose(57.3, -55.2, Math.toRadians(-38.9));
    protected Pose endAutoPose =         new Pose(16.3, -40.7, Math.toRadians(90.0));

    protected Path scorePreload;
    protected PathChain grabPickup1, scorePickup1, rcExcessIntake1, rcExcessIntake2, rcExcessScore, endAuto;

    RedFarAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(60.0, 64.0);
        this.startPose = new Pose(15.8, -63.8, Math.toRadians(90.0));
        this.scorePose = new Pose(3.0, -55.0, Math.toRadians(45.0));
    }

    protected void runDelay(double delay) {
        // Run delay
        if (pathTimer.getElapsedTimeSeconds() >= delay) {
            incrementCycleState();
        }
    }

    protected void runExcessCycle(PathChain excess1, PathChain excess2, PathChain score) {
        switch (pathState) {
            // Disable feeder and excess1
            case 0:
                if (pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
                    hardwareController.gate.setPosition(HardwareController.GATE_CLOSED_ANGLE);
                    follower.followPath(excess1, true);
                    incrementPathState();
                }
                break;
            // excess2
            case 1:
                if (pathTimer.getElapsedTimeSeconds() >= 2.0 && !follower.isBusy()) {
                    follower.followPath(excess2, true);
                    incrementPathState();
                }
                break;
            // Go to score position
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= 1.5 && !follower.isBusy()) {
                    follower.followPath(score, true);
                    incrementPathState();
                }
                break;
            // Get to score position
            case 3:
                if (!follower.isBusy()) incrementCycleState();
                break;
        }
    }

    /**
     * Instanciate and build PathChains
     */
    protected void buildPaths() {

        /* ARTIFACT PRELOAD */

        // Shooting position for preloaded artifacts
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* ARTIFACT SET 1 */

        // Curved intake line for artifact set #1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, postPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), postPickup1Pose.getHeading())
                .build();

        // Shooting position for artifact set #1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(postPickup1Pose, intermediatePickup1Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* RC EXCESS INTAKE PROTOCOL */

        rcExcessIntake1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, rcExcessIntakePose1))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rcExcessIntakePose1.getHeading())
                .build();

        rcExcessIntake2 = follower.pathBuilder()
                .addPath(new BezierCurve(rcExcessIntakePose1, rcExcessIntakePose2))
                .setLinearHeadingInterpolation(rcExcessIntakePose1.getHeading(), rcExcessIntakePose2.getHeading())
                .build();

        rcExcessScore = follower.pathBuilder()
                .addPath(new BezierLine(rcExcessIntakePose2, scorePose))
                .setLinearHeadingInterpolation(rcExcessIntakePose2.getHeading(), scorePose.getHeading())
                .build();

        /* PARKING PROTOCOL */

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endAutoPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endAutoPose.getHeading())
                .build();
    }

    /**
     * Updates the auto paths
     */
    protected void autoPathUpdate(){
        switch (cycleState) {
            // Initial delay
            case 0:
                runDelay(3.0);
                break;

            // Preload
            case 1:
                runPreloadCycle(scorePreload);
                break;

            // Artifact set 1
            case 2:
                runArtifactSetCycle(grabPickup1, scorePickup1);
                break;

            // RC residual #1
            case 3:
                runExcessCycle(rcExcessIntake1, rcExcessIntake2, rcExcessScore);
                break;

            // RC residual #2
            case 4:
                runExcessCycle(rcExcessIntake1, rcExcessIntake2, rcExcessScore);
                break;

            // RC residual #3
            case 5:
                runExcessCycle(rcExcessIntake1, rcExcessIntake2, rcExcessScore);
                break;

            // End-of-auto parking
            case 6:
                runEndAuto(endAuto);
                break;
        }
    }
}


class BlueNearAuto extends RedNearAuto {
    BlueNearAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(-60.0, 64.0);
        this.startPose = new Pose(-37.0, 68.0, Math.toRadians(180.0));
        this.scorePose = new Pose(-15.0, 21.0, Math.toRadians(180.0));
        this.scorePose2 = new Pose(-15.0, 21.0, Math.toRadians(180.0 - 45.0));

        this.postPickup1Pose =         new Pose(-47.5, 16.3, Math.toRadians(180.0));

        this.intermediatePickup2Pose = new Pose(-25.9, -6.5, Math.toRadians(180.0));
        this.postPickup2Pose =         new Pose(-57.0, -5.5,Math.toRadians(180.0));

        this.intermediatePickup3Pose = new Pose(-22.9, -26.8, Math.toRadians(180.0));
        this.postPickup3Pose =         new Pose(-53.0, -29.3, Math.toRadians(180.0));

        this.RCIntermediatePose =      new Pose(-53.7, -15.9, Math.toRadians(180.0 - 28.5));
        this.RCGatePose =              new Pose(-48.6, -0.9, Math.toRadians(180.0));
        this.RCIntakePose =            new Pose(-61.3, -15.0, Math.toRadians(180.0 - 43.7));

        this.endAutoPose =             new Pose(-15.0, 0.0, Math.toRadians(180.0));
    }
}


class BlueFarAuto extends RedFarAuto {
    BlueFarAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(-60.0, 64.0);
        this.startPose = new Pose(-15.8, -63.8, Math.toRadians(90.0));
        this.scorePose = new Pose(-1.0, -50.0, Math.toRadians(180.0 - 45.0));

        this.intermediatePickup1Pose = new Pose(-10.3, -31.0, Math.toRadians(180.0));
        this.postPickup1Pose =    new Pose(-38.0, -29.3, Math.toRadians(180.0));
        this.rcExcessIntakePose1 = new Pose(-53.3, -36.2, Math.toRadians(180.0 + 38.9));
        this.rcExcessIntakePose2 = new Pose(-56.3, -55.2, Math.toRadians(180.0 + 38.9));
        this.endAutoPose =        new Pose(-16.3, -40.7, Math.toRadians(90.0));
    }
}