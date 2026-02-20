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

import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.state.HardwareController;


@Config
@Autonomous(name = "Autonomous", group = "League Tournament")
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

    public static double FEED_DURATION      = 1.0;
    public static double RC_GATE_DURATION   = 0.0;
    public static double RC_INTAKE_DURATION = 1.5;
    public static double READY_DURATION     = 0.5;
    public static double FLYWHEEL_ACCEPTED_ERROR = 1.0; // RPS

    protected Pose goalPose =  new Pose(60.0, 60.0);
    protected Pose startPose = new Pose(40.2, 60.9, Math.toRadians(90.0));
    protected Pose scorePose = new Pose(24.0, 10.8, Math.toRadians(0.0));

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
    }

    @Override
    public final void init_loop() { displayTelemetryMessage(); }

    @Override
    public final void start() {
        pathTimer.resetTimer();

        hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
    }

    @Override
    public final void loop() {
        // Loop movements of robot
        follower.update();
        autoPathUpdate();

        // Perform turret updates
        if (cycleState < 5) {
            hardwareController.updateTurret(follower, goalPose, opmodeTimer.getElapsedTimeSeconds());
        }
        // Log telemetry
        updateTelemetry();
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
                double flywheelRPS = hardwareController.flywheelA.getVelocity() / (HardwareController.FLYWHEEL_TICKS_PER_DEGREE * 360.0);
                if (hardwareController.targetSpeed - FLYWHEEL_ACCEPTED_ERROR <= flywheelRPS) {
                    hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
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
                    hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
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
                if (!follower.isBusy()) incrementPathState();
                break;
            // Shoot artifacts
            case 3:
                if (pathTimer.getElapsedTimeSeconds() >= READY_DURATION) {
                    hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                    incrementCycleState();
                }
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
                    hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
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
                if (!follower.isBusy()) incrementPathState();
                break;
            // Shoot artifacts
            case 6:
                if (pathTimer.getElapsedTimeSeconds() >= READY_DURATION) {
                    hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                    incrementCycleState();
                }
                break;
        }
    }

    /**
     * Run the end auto policy
     */
    protected void runEndAuto(PathChain endAuto) {
        if (pathState == 0 && pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
            hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
            follower.followPath(endAuto, true);
            incrementCycleState();

            // Disable all motors
            hardwareController.gate.setPosition(0.5);
            hardwareController.intake.setPower(0.0);
            hardwareController.transfer.setPower(0.0);

            // Reset turret
            hardwareController.flywheelA.setPower(0.0);
            hardwareController.updateTurretTarget(0.0);

            incrementCycleState();
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

        packet.put("Position (In)", follower.getPose());
        packet.put("Velocity (In/Sec)", follower.getVelocity());
        packet.put("Flywheel Velocity (Rotations/Sec)", hardwareController.flywheelA.getVelocity() / (HardwareController.FLYWHEEL_TICKS_PER_DEGREE * 360));

        packet.put("Flywheel Target Speed (RPS)", hardwareController.targetSpeed);
        packet.put("Turret Target Angle (Degrees)", hardwareController.turretAngle);

        dashboard.sendTelemetryPacket(packet);
    }
}


class RedNearAuto extends DebuggerAuto {
    protected Pose postPickup1Pose =         new Pose(54.7, 8.8, Math.toRadians(0.0));

    protected Pose intermediatePickup2Pose = new Pose(25.9, -16, Math.toRadians(0.0));
    protected Pose postPickup2Pose =         new Pose(58.5, -15.5,Math.toRadians(0.0));

    protected Pose intermediatePickup3Pose = new Pose(19.9, -43.1, Math.toRadians(0.0));
    protected Pose postPickup3Pose =         new Pose(61.3, -39.1, Math.toRadians(0.0));

    protected Pose RCIntermediatePose =      new Pose(61.7, -20.9, Math.toRadians(28.5));
    protected Pose RCGatePose =              new Pose(58.9, -12.8, Math.toRadians(19.5));
    protected Pose RCIntakePose =            new Pose(63.0, -18.1, Math.toRadians(50.9));

    protected Pose endAutoPose =             new Pose(47.8, 0.0, Math.toRadians(90));

    protected Path scorePreload;
    protected PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, openGateRC, intakeRC, scoreRC, endAuto;

    RedNearAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(60.0, 60.0);
        this.startPose = new Pose(40.2, 60.9, Math.toRadians(90.0));
        this.scorePose = new Pose(24.0, 10.8, Math.toRadians(0.0));
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
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose.getHeading())
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
    protected Pose postPickup1Pose =    new Pose(61.5, -61.0, Math.toRadians(0.0));
    protected Pose rcExcessIntakePose = new Pose(61.5, -61.0, Math.toRadians(0.0));
    protected Pose endAutoPose =        new Pose(36.0, -63.0, Math.toRadians(90.0));

    protected Path scorePreload;
    protected PathChain grabPickup1, scorePickup1, rcExcessIntake, rcExcessScore, endAuto;

    RedFarAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(60.0, 60.0);
        this.startPose = new Pose(12.0, -66.7, Math.toRadians(90.0));
        this.scorePose = new Pose(17.0, -58.0, Math.toRadians(0.0));
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
                .addPath(new BezierLine(postPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* RC EXCESS INTAKE PROTOCOL */

        rcExcessIntake = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, rcExcessIntakePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), rcExcessIntakePose.getHeading())
                .build();

        rcExcessScore = follower.pathBuilder()
                .addPath(new BezierLine(rcExcessIntakePose, scorePose))
                .setLinearHeadingInterpolation(rcExcessIntakePose.getHeading(), scorePose.getHeading())
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

            // Artifact set 1
            case 1:
                runArtifactSetCycle(grabPickup1, scorePickup1);
                break;

            // RC residual #1
            case 2:
                runArtifactSetCycle(rcExcessIntake, rcExcessScore);
                break;

            // RC residual #2
            case 3:
                runArtifactSetCycle(rcExcessIntake, rcExcessScore);
                break;

            // RC residual #3
            case 4:
                runArtifactSetCycle(rcExcessIntake, rcExcessScore);
                break;

            // End-of-auto parking
            case 5:
                runEndAuto(endAuto);
                break;
        }
    }
}


class BlueNearAuto extends RedNearAuto {
    BlueNearAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(-60.0, 60.0);
        this.startPose = new Pose(-40.2, 60.9, Math.toRadians(90.0));
        this.scorePose = new Pose(-24.0, 10.8, Math.toRadians(180.0));

        this.postPickup1Pose =         new Pose(-54.7, 8.8, Math.toRadians(180.0));

        this.intermediatePickup2Pose = new Pose(-25.9, -16, Math.toRadians(180.0));
        this.postPickup2Pose =         new Pose(-58.5, -15.5,Math.toRadians(180.0));

        this.intermediatePickup3Pose = new Pose(-19.9, -43.1, Math.toRadians(180.0));
        this.postPickup3Pose =         new Pose(-61.3, -39.1, Math.toRadians(180.0));

        this.RCIntermediatePose =      new Pose(-61.7, -20.9, Math.toRadians(180.0 - 28.5));
        this.RCGatePose =              new Pose(-58.9, -12.8, Math.toRadians(180.0 - 19.5));
        this.RCIntakePose =            new Pose(-63.0, -18.1, Math.toRadians(180.0 - 50.9));

        this.endAutoPose =             new Pose(-47.8, 0.0, Math.toRadians(90.0));
    }
}


class BlueFarAuto extends RedFarAuto {
    BlueFarAuto() {
        super();
        // Reset poses
        this.goalPose =  new Pose(-60.0, 60.0);
        this.startPose = new Pose(-12.0, -63.0, Math.toRadians(90.0));
        this.scorePose = new Pose(-17.0, -58.0, Math.toRadians(180.0));

        this.postPickup1Pose =    new Pose(-61.5, -67.7, Math.toRadians(180.0));
        this.rcExcessIntakePose = new Pose(-61.5, -67.7, Math.toRadians(180.0));
        this.endAutoPose =        new Pose(-36.0, -67.7, Math.toRadians(90.0));
    }
}