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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;

import java.util.List;
import java.util.function.Supplier;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LM2 TeleOp Blue", group="League Meet 2")
public class TeleOpBlue extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Limelight3A limelight;
    private LLResult result;
    private HardwareController hardwareController;

    private Follower follower;
    public static Pose startingPose;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryManager;

    private boolean slowMode = false;
    private boolean debugTelemetry = false;
    private boolean autoAiming = true;

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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        // Set poll rate
        limelight.setPollRateHz(100);
        limelight.start();
        /*
        pathChain = () -> follower.pathBuilder()
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
         */

        // Wait until driver presses "start"
        waitForStart();
        runtime.reset();

        // Enable flywheel to start
        hardwareController.toggleFlywheel(true);
        follower.startTeleOpDrive(true);

        // Functional loop of OpMode
        while (opModeIsActive()) {

            follower.update();

            // Normal driving mode
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // field-centric
            );
                // Precision driving mode
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                    -gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                    -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                    true // field-centric
            );

            /* NON-DRIVING CONTROLS */

            // Toggle slow mode
            if (gamepad1.aWasPressed()) slowMode = !slowMode;

            // Toggle turret auto aiming
            if (gamepad1.xWasPressed()) autoAiming = !autoAiming;

            // Toggle debug telemetry
            if (gamepad1.bWasPressed()) debugTelemetry = !debugTelemetry;

            // Outtake
            if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Cycle to outtake if not done
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(
                        HardwareController.INTAKE_POWER
                );
                hardwareController.transfer.setPower(
                        HardwareController.TRANSFER_POWER
                );
            }

            // Power the intake
            if (gamepad1.right_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(
                        HardwareController.INTAKE_POWER
                );
                hardwareController.transfer.setPower(
                        HardwareController.TRANSFER_POWER
                );
            }

            // Feed the artifacts
            if (gamepad1.right_bumper) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Switch intake mode to [intake] if needed
                if (!hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then feed and intake
                hardwareController.transfer.setPower(
                        HardwareController.TRANSFER_POWER
                );
                hardwareController.intake.setPower(
                        HardwareController.INTAKE_POWER
                );
            }

            // Ensure motors are properly disabled when not in use
            if (gamepad1.right_trigger < TRIGGER_THRESHOLD && gamepad1.left_trigger < TRIGGER_THRESHOLD) {
                hardwareController.intake.setPower(0.0);
                // Only disable transfer when RB is not pressed
                if (gamepad1.right_bumper) {
                    hardwareController.transfer.setPower(0.0);
                }
            }

            // Turret auto-aiming
            if (autoAiming) {
                result = limelight.getLatestResult();
                // Proceed if any fiducials have been detected
                if (result != null && result.isValid()) {
                    // Iterate fiducials
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        int id = fiducial.getFiducialId(); // The ID number of the fiducial
                        if (id == 20) { // ID = 20 corresponds to the blue goal
                            // Get values for computation
                            double tA = fiducial.getTargetArea();
                            double distance = getTargetDist(tA);
                            // Compute and update flywheel position
                            hardwareController.updateTurretTarget(fiducial.getTargetXDegrees());
                            hardwareController.updateFlywheel(distance);
                        } else {
                            hardwareController.resetTurret();
                        }
                    }
                }
            }
            hardwareController.toggleFlywheel(true);

            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        // Debug telemetry
        if (debugTelemetry) {
            telemetry.addData("Position", "(%.3f, %.3f, %.3f)", follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading() * (180 / Math.PI));
            telemetry.addData("Velocity", "(%.3f, %.3f, %.3f)", follower.getVelocity().getXComponent(), follower.getVelocity().getYComponent(), follower.getAngularVelocity() * (180 / Math.PI));
            telemetry.addData("Flywheel Velocity", hardwareController.getFlywheelVelocity());
            telemetry.addLine();
        }

        // Statuses
        telemetry.addData("Auto Aiming", autoAiming);
        telemetry.addData("Precision Mode", slowMode);
        telemetry.addLine();

        // Controls
        telemetry.addLine("A - Precision Mode");
        telemetry.addLine("X - Auto Aiming");
        telemetry.addLine("B - Show Debug Telemetry\n");

        telemetry.addLine("LT - Intake Power");
        telemetry.addLine("LB - Intake Direction\n");

        telemetry.addLine("RT - Conditional Feed");
        telemetry.addLine("RB - Flywheel");

        telemetry.update();
    }

    public double getTargetDist(double targetArea) {
        double scale = 14.76;
        return scale/Math.sqrt(targetArea);
    }
}
