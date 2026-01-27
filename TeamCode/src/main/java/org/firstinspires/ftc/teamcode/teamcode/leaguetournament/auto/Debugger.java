/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.auto;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
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

@Autonomous(name="Auto Debugger", group="League Meet 2")
public class Debugger extends LinearOpMode {
    @IgnoreConfigurable
    private Timer pathTimer, opmodeTimer;
    @IgnoreConfigurable
    private Follower follower;
    @IgnoreConfigurable
    private HardwareController hardwareController;
    @IgnoreConfigurable
    private TelemetryManager telemetryM;
    private int pathState;

    private static final double FEED_DURATION      = 1.0;
    private static final double RAMP_CAMP_DURATION = 3.0;

    private static Pose goalPose =                new Pose(60.0, 60.0);
    private static Pose startPose =               new Pose(0.0, 0.0, Math.toRadians(0.0));
    private static Pose scorePose =               new Pose(0.0, 0.0, 0.0);

    private static Pose intermediatePickup1Pose = new Pose(0.0, 0.0, 0.0);
    private static Pose postPickup1Pose =         new Pose(0.0, 0.0, 0.0);

    private static Pose intermediatePickup2Pose = new Pose(0.0, 0.0, 0.0);
    private static Pose postPickup2Pose =         new Pose(0.0, 0.0,0.0);

    private static Pose intermediatePickup3Pose = new Pose(0.0, 0.0, 0.0);
    private static Pose postPickup3Pose =         new Pose(0.0, 0.0, 0.0);

    private static Pose openGatePose =            new Pose(0.0, 0.0, 0.0);
    private static Pose rampCampPose =            new Pose(0.0, 0.0, 0.0);

    private static Pose endAutoPose =             new Pose(50.0, 0.0, Math.toRadians(-90.0));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, openGateRC, intakeRC, scoreRC, endAuto;

    @Override
    public void runOpMode() {
        // Pedro objects
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = ConstantsEpsilon.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Hardware controller for mechanism access
        hardwareController = new HardwareController(hardwareMap);
        hardwareController.resetTurret();

        waitForStart();

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
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getPose().getHeading());

        /* ARTIFACT SET 1 */

        // Curved intake line for artifact set #1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup1Pose, postPickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), postPickup1Pose.getHeading())
                .build();

        // Shooting position for artifact set #1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose.getPose().getHeading())
                .build();

        /* ARTIFACT SET 2 */

        // Curved intake line for artifact set #2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup2Pose, postPickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), postPickup2Pose.getHeading())
                .build();

        // Shooting position for artifact set #2
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(postPickup2Pose, intermediatePickup2Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup2Pose.getHeading(), scorePose.getPose().getHeading())
                .build();

        /* ARTIFACT SET 3 */

        // Curved intake line for artifact set #3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup3Pose, postPickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), postPickup3Pose.getHeading())
                .build();

        // Shooting position for artifact set #3
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup3Pose.getHeading(), scorePose.getPose().getHeading())
                .build();

        /* RAMP CAMP PROTOCOL */

        // Curved gate open per G418
        openGateRC = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intermediatePickup2Pose, openGatePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGatePose.getHeading())
                .build();

        // Ramp camp
        intakeRC = follower.pathBuilder()
                .addPath(new BezierLine(openGatePose, rampCampPose))
                .setLinearHeadingInterpolation(openGatePose.getHeading(), rampCampPose.getHeading())
                .build();

        // Shooting position for artifact set #1
        scoreRC = follower.pathBuilder()
                .addPath(new BezierCurve(rampCampPose, intermediatePickup2Pose, scorePose))
                .setLinearHeadingInterpolation(rampCampPose.getHeading(), scorePose.getPose().getHeading())
                .build();

        /* PARKING PROTOCOL */

        endAuto = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endAutoPose))
                .setLinearHeadingInterpolation(scorePose.getPose().getHeading(), endAutoPose.getHeading())
                .build();
    }

    /**
     * Updates the auto paths
     */
    private void autoPathUpdate(){
        switch (pathState) {
            // Preload
            // Go to score position
            case 0:
                follower.followPath(scorePreload, true);
                setPathState(1, !follower.isBusy());
                break;
            // Feed for duration
            case 1:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(10, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // Artifact set 2
            // Disable feeder and intake artifacts
            case 10:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(grabPickup2, true);
                setPathState(11, !follower.isBusy());
                break;
            // Go to score position
            case 11:
                follower.followPath(scorePickup2, true);
                setPathState(12, !follower.isBusy());
                break;
            // Feed for duration
            case 12:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(20, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // Ramp camp 1
            // Disable feeder and open gate
            case 20:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(openGateRC, true);
                setPathState(21, !follower.isBusy());
                break;
            // Go to RC intake position
            case 21:
                follower.followPath(intakeRC, true);
                setPathState(22, !follower.isBusy());
                break;
            // Pause to intake
            case 22:
                setPathState(23, pathTimer.getElapsedTimeSeconds() >= RAMP_CAMP_DURATION);
                break;
            // Go to score position
            case 23:
                follower.followPath(scoreRC, true);
                setPathState(24, !follower.isBusy());
                break;
            // Feed for duration
            case 24:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(30, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // Ramp camp 2
            // Disable feeder and open gate
            case 30:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(openGateRC, true);
                setPathState(31, !follower.isBusy());
                break;
            // Go to RC intake position
            case 31:
                follower.followPath(intakeRC, true);
                setPathState(32, !follower.isBusy());
                break;
            // Pause to intake
            case 32:
                setPathState(33, pathTimer.getElapsedTimeSeconds() >= RAMP_CAMP_DURATION);
                break;
            // Go to score position
            case 33:
                follower.followPath(scoreRC, true);
                setPathState(34, !follower.isBusy());
                break;
            // Feed for duration
            case 34:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(40, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // Ramp camp 1
            // Disable feeder and open gate
            case 40:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(openGateRC, true);
                setPathState(41, !follower.isBusy());
                break;
            // Go to RC intake position
            case 41:
                follower.followPath(intakeRC, true);
                setPathState(42, !follower.isBusy());
                break;
            // Pause to intake
            case 42:
                setPathState(43, pathTimer.getElapsedTimeSeconds() >= RAMP_CAMP_DURATION);
                break;
            // Go to score position
            case 43:
                follower.followPath(scoreRC, true);
                setPathState(44, !follower.isBusy());
                break;
            // Feed for duration
            case 44:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(50, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // Artifact set 1
            // Disable feeder and intake artifacts
            case 50:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(grabPickup1, true);
                setPathState(51, !follower.isBusy());
                break;
            // Go to score position
            case 51:
                follower.followPath(scorePickup1, true);
                setPathState(52, !follower.isBusy());
                break;
            // Feed for duration
            case 52:
                hardwareController.gate.setPosition(HardwareController.OPEN_ANGLE);
                setPathState(60, pathTimer.getElapsedTimeSeconds() >= FEED_DURATION);
                break;


            // End-of-auto parking
            case 60:
                hardwareController.gate.setPosition(HardwareController.CLOSED_ANGLE);
                follower.followPath(endAuto, true);
                setPathState(-1, !follower.isBusy());
                break;
        }
    }

    /**
     * Advanced to a new path state if the given condition is met
     *
     * @param pathState new path state
     * @param condition condition to be met
     */
    private void setPathState(int pathState, boolean condition) {
        // Attempt path state reset
        // Usually, condition will be follower has reached pose
        if (condition) {
            this.pathState = pathState;
            pathTimer.resetTimer();
        }
    }

    /**
     * Updates panels telemetry
     */
    private void updateTelemetry() {
        // Write telemetry
        telemetryM.addData("Path State", pathState);
        telemetryM.addData("Path Timer", pathTimer.getElapsedTime());
        telemetryM.addData("Target Pose", follower.getCurrentPath().getPose(1.0));

        telemetryM.addData("Position (In)", follower.getPose());
        telemetryM.addData("Velocity (In/Sec)", follower.getVelocity());
        telemetryM.addData("Flywheel Velocity (Degrees/Sec)", hardwareController.turretFlywheel.getVelocity(AngleUnit.DEGREES));

        telemetryM.update();
    }
}