/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.LimelightController;

import java.util.function.Supplier;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LM2 TeleOp Red", group="League Meet 2")
public class TeleOpRed extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private LimelightController limelightController;
    private HardwareController hardwareController;

    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryManager;

    private boolean slowMode = false;
    private boolean flywheelOn = false;
    private boolean debugTelemetry = false;
    private DcMotorSimple.Direction intakeMode = DcMotorSimple.Direction.FORWARD;

    private double SLOW_MODE_MULTIPLIER = 0.25;
    private double TRIGGER_THRESHOLD = 0.05;

    @Override
    public void runOpMode() {
        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap);

        // Pedro objects
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryManager = PanelsTelemetry.INSTANCE.getTelemetry();

        /*
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
         */

        // Wait until driver presses "start"
        waitForStart();
        runtime.reset();

        follower.startTeleOpDrive(true);

        // Functional loop of OpMode
        while (opModeIsActive()) {

            follower.update();

            // Normal driving mode
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    false // field-centric
            );
            // Precision driving mode
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                    -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                    -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                    false // field-centric
            );

            /* NON-DRIVING CONTROLS */

            // Toggle slow mode
            if (gamepad1.aWasPressed()) slowMode = !slowMode;

            // Toggle debug telemetry
            if (gamepad1.bWasPressed()) debugTelemetry = !debugTelemetry;

            // Toggle flywheel
            if (gamepad1.rightBumperWasPressed()) {
                flywheelOn = !flywheelOn;
                // Set power accordingly
                hardwareController.toggleFlywheel(flywheelOn);
            }

            // Toggle the intake direction
            if (gamepad1.leftBumperWasPressed()) {
                intakeMode = (intakeMode.equals(DcMotorSimple.Direction.REVERSE)) ? DcMotorSimple.Direction.FORWARD : DcMotorSimple.Direction.REVERSE;
                // Set direction accordingly
                hardwareController.intake.setDirection(intakeMode);
            }

            // Power the intake
            // When trigger is held, intake
            if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode if needed
                if (!hardwareController.intake.getDirection().equals(intakeMode)) {
                    hardwareController.intake.setDirection(intakeMode);
                }
                // Then power intake and
                hardwareController.intake.setPower(
                        HardwareController.INTAKE_POWER
                );
                hardwareController.transfer.setPower(
                        HardwareController.TRANSFER_POWER
                );
            }

            // Feed the artifacts
            // When trigger is held and flywheel velocity is acceptable, feed
            if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Switch intake mode to [intake] if needed
                if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then feed and intake
                boolean inRange = hardwareController.conditionalFeed();
                if (inRange) hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
            }

            // Ensure motors are properly disabled when not in use
            if (gamepad1.right_trigger < TRIGGER_THRESHOLD && gamepad1.left_trigger < TRIGGER_THRESHOLD) {
                hardwareController.transfer.setPower(0.0);
                hardwareController.intake.setPower(0.0);
            }


            // Turret auto-aiming
            LLResultTypes.FiducialResult fiducial = limelightController.updateResult(24);
            if(fiducial != null) {
                double distance = limelightController.getTargetDist(fiducial.getTargetArea());
                hardwareController.updateTurretTarget(fiducial.getTargetXDegrees());
                hardwareController.updateFlywheel(distance);
            }

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        // Debug telemetry
        if (debugTelemetry) {
            telemetry.addData("POSITION", follower.getPose());
            telemetry.addData("VELOCITY", follower.getVelocity());
            telemetry.addData("FLYWHEEL VELOCITY", hardwareController.getFlywheelVelocity());
        }

        // Controls
        telemetry.addLine("A - Precision Mode");
        telemetry.addLine("B - Show Debug Telemetry\n");

        telemetry.addLine("LT - Intake Power");
        telemetry.addLine("LB - Intake Direction\n");

        telemetry.addLine("RT - Conditional Feed");
        telemetry.addLine("RB - Flywheel");

        telemetry.update();
    }
}
