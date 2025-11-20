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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet1;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.MotorDriverPID;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet1.utils.RRMotorActions;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
 * This OpMode runs a simple 9 ball auto
 */

@Autonomous(name="League Meet 1 (Auto)", group="League Meet 1")
public class AutoLeagueMeet1 extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private RRMotorActions motorActions;

    private DcMotorEx intakeMotor;
    private DcMotorEx transportMotor;
    private DcMotorEx flywheelMotor;

    private static final boolean USE_WEBCAM = true;
    private static final double FEED_DURATION = 5.0;

    @Override
    public void runOpMode() {

        //initAprilTag();

        // Map mechanism motors
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        transportMotor = hardwareMap.get(DcMotorEx.class, "transport");
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "flywheel");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        transportMotor.setDirection(DcMotor.Direction.FORWARD);
        flywheelMotor.setDirection(DcMotor.Direction.REVERSE);

        // Build starter pose and odometry drive
        Pose2d start = new Pose2d(0.0, 0.0, Math.toRadians(0.0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, start);

        // Instanciate new action handler
        motorActions = new RRMotorActions(transportMotor);

        // Always power the flywheel and intake (for testing)
        intakeMotor.setVelocity(
                MotorDriverPID.toTPS(MotorDriverPID.INTAKE_RPS)
        );
        flywheelMotor.setVelocity(
                MotorDriverPID.toTPS(MotorDriverPID.FLYWHEEL_RPS)
        );

        /*
         * =====================
         * Determine trajectories for all stages of (9-ball) auto
         * =====================
         */

        // Build initial batch cycle (no intake)
        TrajectoryActionBuilder traj0 = drive.actionBuilder(start)
                // Linear position to shoot first batch
                .lineToX(4.0);

        // Build first intake batch cycle
        TrajectoryActionBuilder traj1 = drive.actionBuilder(start)
                // Orient to intake
                .turn(Math.toRadians(127))
                // Align with first set of balls
                .strafeTo(new Vector2d(14.5, 0.0))
                // Intake and return
                .strafeTo(new Vector2d(0.0, 35.0))
                .strafeTo(new Vector2d(0.0, -35.0))
                .strafeTo(new Vector2d(-14.5, 0.0))
                // Turn to face goal
                .turn(Math.toRadians(127));

        // Build second intake batch cycle
        TrajectoryActionBuilder traj2 = drive.actionBuilder(start)
                // Orient to intake
                .turn(Math.toRadians(127))
                // Align with first set of balls
                .strafeTo(new Vector2d(14.5, 0.0))
                // Intake and return
                .strafeTo(new Vector2d(0.0, 35.0))
                .strafeTo(new Vector2d(0.0, -35.0))
                .strafeTo(new Vector2d(-14.5, 0.0))
                // Turn to face goal
                .turn(Math.toRadians(127));

        // Compile actions
        SequentialAction sequentialAction = new SequentialAction(
                // Follow pre-launch trajectory
                traj0.build(),
                // Feed for set duration
                motorActions.feedFlywheel(FEED_DURATION),
                // Follow second set trajectory and feed
                traj1.build(),
                motorActions.feedFlywheel(FEED_DURATION),
                // Follow final set trajectory
                traj2.build(),
                motorActions.feedFlywheel(FEED_DURATION)
        );

        // Add preliminary telemetry data
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Quit auto if stop is requested
        if (isStopRequested()) return;

        // Chain trajectories
        Actions.runBlocking(
                sequentialAction
        );

        intakeMotor.setVelocity(0.0);
        flywheelMotor.setVelocity(0.0);

        /*

        // Basic AprilTag code to adapt at a later date

        // Begin streaming
        visionPortal.resumeStreaming();

        // Stop execution if it is requested
        if (isStopRequested()) {
            visionPortal.close();
            return;
        }

        // Process apriltag and derive pose
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        // Loop apriltags
        for (AprilTagDetection detection : currentDetections) {
            if (detection.id == 0) {

            }
        }

        // Quit streaming
        visionPortal.close();

        */
    }

    private void initAprilTag() {
        // Initialize the apriltag processor and vision portal
        aprilTag = new AprilTagProcessor.Builder().build();
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera to use webcam
        builder.setCamera(hardwareMap.get(WebcamName.class, "webcam"));

        // Set and enable the processor and vision portal
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }
}