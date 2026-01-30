package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.epsilon.ConstantsEpsilon;
import org.firstinspires.ftc.teamcode.teamcode.leaguetournament.HardwareController;

@Config
@Autonomous(name="Auto Debugger", group="League Meet 2")
public class AutoPackage extends LinearOpMode {
    private Timer pathTimer, opmodeTimer;
    private Follower follower;
    private HardwareController hardwareController;
    private TelemetryPacket packet;
    private FtcDashboard dashboard;
    private int pathState, cycleState = 0;

    public static double FEED_DURATION      = 1.0;
    public static double RC_GATE_DURATION   = 1.0;
    public static double RC_INTAKE_DURATION = 2.0;
    public static double FLYWHEEL_ACCEPTED_ERROR = 2.0; // RPS

    private static Pose goalPose =                new Pose(60.0, 60.0);
    private static Pose startPose =               new Pose(40.2, 60.9, Math.toRadians(90.0));
    private static Pose scorePose =               new Pose(25.0, 8.8, Math.toRadians(0.0));

    private static Pose postPickup1Pose =         new Pose(56.7, 8.8, Math.toRadians(0.0));

    private static Pose intermediatePickup2Pose = new Pose(25.9, -16, Math.toRadians(0.0));
    private static Pose postPickup2Pose =         new Pose(58.5, -15.5,Math.toRadians(0.0));

    private static Pose intermediatePickup3Pose = new Pose(25.9, -39.1, Math.toRadians(0.0));
    private static Pose postPickup3Pose =         new Pose(61.3, -39.1, Math.toRadians(0.0));

    private static Pose openGatePose =            new Pose(61.2, -13.0, Math.toRadians(31.8));
    private static Pose rampCampPose =            new Pose(61.4, -18.5, Math.toRadians(60.5));

    private static Pose endAutoPose =             new Pose(47.8, 0.0, Math.toRadians(90));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, openGateRC, intakeRC, scoreRC, endAuto;

    @Override
    public void runOpMode() {
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

        // Ramp up flywheel speed prior to shooting preload
        hardwareController.updateTurret(follower, goalPose, 0.0);

        waitForStart();
        pathTimer.resetTimer();

        hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);

        // Quit auto if stop is requested
        if (isStopRequested()) return;

        // Functional loop of OpMode
        while (opModeIsActive()) {
            // Loop movements of robot
            follower.update();
            autoPathUpdate();

            // Perform turret updates
            hardwareController.updateTurret(follower, goalPose, opmodeTimer.getElapsedTimeSeconds());

            // Log telemetry
            updateTelemetry();

            if (isStopRequested()) {
                // Disable all motors
                hardwareController.gate.setPosition(0.5);
                hardwareController.intake.setPower(0.0);
                hardwareController.transfer.setPower(0.0);
                follower.breakFollowing();

                // Reset turret
                hardwareController.turretFlywheel.setPower(0.0);
                hardwareController.updateTurretTarget(0.0);
                break;
            }
        }

    }

    /**
     * Instanciate and build PathChains
     */
    private void buildPaths() {

        /* ARTIFACT PRELOAD */

        // Shooting position for preloaded artifacts
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setTangentHeadingInterpolation();

        /* ARTIFACT SET 1 */

        // Curved intake line for artifact set #1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, postPickup1Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Shooting position for artifact set #1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup1Pose, scorePose))
                .setTangentHeadingInterpolation()
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
                .setTangentHeadingInterpolation()
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
                .setTangentHeadingInterpolation()
                .build();

        /* RAMP CAMP PROTOCOL */

        // Curved gate open per G418
        openGateRC = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup2Pose, openGatePose))
                .setTangentHeadingInterpolation()
                .build();

        /*// Ramp camp
        intakeRC = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, rampCampPose))
                .setTangentHeadingInterpolation()
                .build();*/

        // Shooting position for artifact set #1
        scoreRC = follower.pathBuilder()
                .addPath(new BezierCurve(rampCampPose, intermediatePickup2Pose, scorePose))
                .setTangentHeadingInterpolation()
                .build();

        /* PARKING PROTOCOL */

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endAutoPose))
                .setTangentHeadingInterpolation()
                .build();
    }

    /**
     * Updates the auto paths
     */
    private void autoPathUpdate(){
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
                runRCCycle(openGateRC, scoreRC);
                break;

            // RC 2
            case 3:
                runRCCycle(openGateRC, scoreRC);
                break;

            // Artifact set 1
            case 4:
                runArtifactSetCycle(grabPickup2, scorePickup2);
                break;

            // End-of-auto parking
            case 5:
                if (pathTimer.getElapsedTimeSeconds() >= FEED_DURATION) {
                    hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                    follower.followPath(endAuto, true);
                    incrementCycleState();
                }
                break;
        }
    }

    /**
     * Run preload cycle
     *
     * @param score scoring path
     */
    private void runPreloadCycle(Path score) {
        switch (pathState) {
            // Go to score position
            case 0:
                follower.followPath(score);
                incrementPathState();
                break;
            // Feed for duration
            case 2:
                // Advance if flywheel is up to speed
                double flywheelRPS = hardwareController.turretFlywheel.getVelocity() / HardwareController.FLYWHEEL_TICKS_PER_DEGREE;
                if (hardwareController.targetSpeed - FLYWHEEL_ACCEPTED_ERROR <= flywheelRPS && hardwareController.targetSpeed + FLYWHEEL_ACCEPTED_ERROR >= flywheelRPS) {
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
    private void runArtifactSetCycle(PathChain grabPickup, PathChain score) {
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
            // Feed for duration
            case 2:
                if (!follower.isBusy()) {
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
     * @param score scoring path
     */
    private void runRCCycle(PathChain openGate, PathChain score) {
        switch (pathState) {
            // Disable feeder and open gate
            case 0:
                if (pathTimer.getElapsedTimeSeconds() >= FEED_DURATION && !follower.isBusy()) {
                    hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                    follower.followPath(openGate, true);
                    incrementPathState();
                }
                break;
            // Pause to press gate and intake
            case 1:
                if (!follower.isBusy()) incrementPathState();
            // Go to score position
            case 2:
                if (pathTimer.getElapsedTimeSeconds() >= RC_INTAKE_DURATION) {
                    follower.followPath(score, true);
                    incrementPathState();
                }
                break;
            // Feed for duration
            case 3:
                if (!follower.isBusy()) {
                    hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                    incrementCycleState();
                }
                break;
        }
    }

    /**
     * Advanced to the next cycle
     */
    private void incrementCycleState() {
        cycleState++;
        // Cycle state resets always come with path state resets
        pathState = 0;
        pathTimer.resetTimer();
    }

    /**
     * Advanced to the next path state
     */
    private void incrementPathState() {
        pathState++;
        pathTimer.resetTimer();
    }

    /**
     * Updates panels telemetry
     */
    private void updateTelemetry() {
        // Write telemetry
        packet.put("Path State", pathState);
        packet.put("Path Timer", pathTimer.getElapsedTime());
        packet.put("Target Pose", follower.getCurrentPath().getPose(1.0));

        packet.put("Position (In)", follower.getPose());
        packet.put("Velocity (In/Sec)", follower.getVelocity());
        packet.put("Flywheel Velocity (Degrees/Sec)", hardwareController.turretFlywheel.getVelocity(AngleUnit.DEGREES));

        dashboard.sendTelemetryPacket(packet);
    }
}