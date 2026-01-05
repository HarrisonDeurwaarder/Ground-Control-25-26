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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.auto;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;

@Autonomous(name="LM2 Auto (Close, Red)", group="League Meet 2")
public class AutoCloseRed extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private HardwareController hardwareController;
    private TelemetryManager telemetryM;
    private int pathState;

    private static final int FEED_DURATION = 5000;

    private final Pose startPose =       new Pose(49.534, 61.089, 37.7 * (Math.PI / 180));
    private final Pose scorePose =       new Pose(7.107, 21.663, 225 * (Math.PI / 180)); // Default scoring pose (euclidean coordinates will be supplanted)
    private final Pose prePickup1Pose =  new Pose(25.0, 12.0, 0.0);
    private final Pose postPickup1Pose = new Pose(35.0, 12.0, 0.0);
    private final Pose prePickup2Pose =  new Pose(25.0, -12.0, 0.0);
    private final Pose postPickup2Pose = new Pose(35.0, -12.0, 0.0);
    private final Pose prePickup3Pose =  new Pose(25.0, -36.0, 0.0);
    private final Pose postPickup3Pose = new Pose(35.0, -36.0, 0.0);

    private Path scorePreload;
    private PathChain readyPickup1, grabPickup1, scorePickup1, readyPickup2, grabPickup2, scorePickup2, readyPickup3, grabPickup3, scorePickup3;

    @Override
    public void runOpMode() {

        // Pedro objects
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Hardware controller for mechanism access
        hardwareController = new HardwareController(hardwareMap, startPose);

        waitForStart();
        runtime.reset();

        // Start mechanism motors upon start
        hardwareController.resetTurret();
        hardwareController.turretFlywheel.setVelocity(HardwareController.toTPS(HardwareController.DEFAULT_FLYWHEEL_RPS));
        hardwareController.intake.setPower(HardwareController.INTAKE_POWER);

        // Quit auto if stop is requested
        if (isStopRequested()) return;

        // Functional loop of OpMode
        while (opModeIsActive()) {
            // Loop movements of robot
            follower.update();
            autoPathUpdate();

            // Log telemetry
            updateTelemetry();
        }
    }

    private void buildPaths() {
        // Shooting position for preloaded artifacts
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* ARTIFACT SET 1 */

        // Pre-pickup position for artifact set #1
        readyPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup1Pose.getHeading())
                .build();

        // Colinear intake line for artifact set #1
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup1Pose, postPickup1Pose))
                .setLinearHeadingInterpolation(prePickup1Pose.getHeading(), postPickup1Pose.getHeading())
                .build();

        // Shooting position for artifact set #1
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup1Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* ARTIFACT SET 2 */

        // Pre-pickup position for artifact set #2
        readyPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup2Pose.getHeading())
                .build();

        // Colinear intake line for artifact set #2
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup2Pose, postPickup2Pose))
                .setLinearHeadingInterpolation(prePickup2Pose.getHeading(), postPickup2Pose.getHeading())
                .build();

        // Shooting position for artifact set #2
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup2Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* ARTIFACT SET 3 */

        // Pre-pickup position for artifact set #3
        readyPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, prePickup3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prePickup3Pose.getHeading())
                .build();

        // Colinear intake line for artifact set #3
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(prePickup3Pose, postPickup3Pose))
                .setLinearHeadingInterpolation(prePickup3Pose.getHeading(), postPickup3Pose.getHeading())
                .build();

        // Shooting position for artifact set #3
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(postPickup3Pose, scorePose))
                .setLinearHeadingInterpolation(postPickup3Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    private void autoPathUpdate(){
        switch (pathState) {
            // Score preload position
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            // Score preload
            case 1:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < FEED_DURATION) {
                        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                    } else {
                        hardwareController.transfer.setPower(0.0);
                        setPathState(2);
                    }
                }
                break;


            // Artifact set #1 pre-grab
            case 2:
                // Score first
                follower.followPath(readyPickup1, true);
                setPathState(3);
                break;
            // Artifact set #1 artifact intake
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;
            // Artifact set #1 scoring
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            // Score artifact set #1
            case 5:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < FEED_DURATION) {
                        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                    } else {
                        hardwareController.transfer.setPower(0.0);
                        setPathState(6);
                    }
                }
                break;


            // Artifact set #2 pre-grab
            case 6:
                follower.followPath(readyPickup2, true);
                setPathState(7);
                break;
            // Artifact set #2 artifact intake
            case 7:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup2, true);
                    setPathState(8);
                }
                break;
            // Artifact set #2 scoring
            case 8:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2, true);
                    setPathState(9);
                }
                break;
            // Score artifact set #2
            case 9:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < FEED_DURATION) {
                        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                    } else {
                        hardwareController.transfer.setPower(0.0);
                        setPathState(10);
                    }
                }
                break;


            // Artifact set #1 pre-grab
            case 10:
                if (!follower.isBusy()) {
                    follower.followPath(readyPickup3, true);
                    setPathState(11);
                }
                break;
            // Artifact set #1 artifact intake
            case 11:
                if (!follower.isBusy()) {
                    follower.followPath(grabPickup3, true);
                    setPathState(12);
                }
                break;
            // Artifact set #1 scoring
            case 12:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup3, true);
                    setPathState(13);
                }
                break;
            // Score artifact set #3
            case 13:
                if (!follower.isBusy()) {
                    if (pathTimer.getElapsedTime() < FEED_DURATION) {
                        hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                    } else {
                        hardwareController.transfer.setPower(0.0);
                        hardwareController.turretFlywheel.setPower(0.0);
                        hardwareController.intake.setPower(0.0);
                        setPathState(-1);
                    }
                }
                break;
        }
    }

    private void updateTelemetry() {
        // Write telemetry
        telemetryM.debug("Path State", pathState);
        telemetryM.debug("Path Timer", pathTimer.getElapsedTime());

        telemetryM.debug("Position (In)", follower.getPose());
        telemetryM.debug("Velocity (In/Sec)", follower.getVelocity());
        telemetryM.debug("Flywheel Velocity (Degrees/Sec)", hardwareController.turretFlywheel.getVelocity(AngleUnit.DEGREES));

        telemetry.update();
    }

    private void setPathState(int pathState) {
        this.pathState = pathState;
        pathTimer.resetTimer();
    }
}