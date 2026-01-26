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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.delta.ConstantsDelta;
import org.firstinspires.ftc.teamcode.teamcode.leaguemeet2.utils.HardwareController;

/*
 * COMP READY TELEOP PROCEDURE
 *
 * 1. Remove flywheel toggle conditional / control telemetry
 * 2. Remove team selection block
 * 3. Remove robot centric selection block
 * 4. Replace prestart loop with waitForStart()
 * 5. Remove configurable
*/

@Disabled
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="LM2 TeleOp (Blue, Near)", group="League Meet 2")
public class TeleOpBlueNear extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private HardwareController hardwareController;

    private Follower follower;
    public static Pose startingPose = new Pose(-50.0, 15.0, Math.toRadians(-90.0));
    private TelemetryManager telemetryM;

    private boolean isTeamRed = true;
    private boolean isRobotCentric = false;

    private boolean slowMode = false;
    private boolean flywheelOn = false;

    private double SLOW_MODE_MULTIPLIER = 0.25;
    private double TRIGGER_THRESHOLD = 0.05;

    @Override
    public void runOpMode() {
        // Instanciate controllers
        hardwareController = new HardwareController(hardwareMap, new Pose());
        hardwareController.isRedTeam = false;
        // Pedro objects
        follower = ConstantsDelta.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Set team while waiting for start
        while (!isStarted()) {
            // Only allow team switching/control frame prior to start
            if (gamepad1.xWasPressed()) isTeamRed = !isTeamRed;
            if (gamepad1.bWasPressed()) isRobotCentric = !isRobotCentric;
            // Update telemetry
            telemetry.addData("Team selected (X): ", isTeamRed ? "RED" : "BLUE");
            telemetry.addData("Control frame selected (X): ", isRobotCentric ? "ROBOT" : "FIELD");
            telemetryM.update();
        }
        // Upon start
        runtime.reset();

        follower.startTeleOpDrive(true);

        // Functional loop of OpMode
        while (opModeIsActive()) {

            follower.update();

            // Normal driving mode
            if (!slowMode) follower.setTeleOpDrive(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    isRobotCentric
            );
            // Precision driving mode
            else follower.setTeleOpDrive(
                    gamepad1.left_stick_y * SLOW_MODE_MULTIPLIER,
                    gamepad1.left_stick_x * SLOW_MODE_MULTIPLIER,
                    -gamepad1.right_stick_x * SLOW_MODE_MULTIPLIER,
                    isRobotCentric
            );

            /* NON-DRIVING CONTROLS */

            // Toggle slow mode
            if (gamepad1.aWasPressed()) slowMode = !slowMode;

            // Toggle auto-aiming
            if (gamepad1.xWasPressed()) hardwareController.autoAiming = !hardwareController.autoAiming;

            // Toggle flywheel
            if (gamepad1.rightBumperWasPressed()) {
                flywheelOn = !flywheelOn;
                // Set power accordingly
                hardwareController.turretFlywheel.setVelocity(
                        (flywheelOn) ? HardwareController.toTPS(HardwareController.DEFAULT_FLYWHEEL_RPS) : 0.0
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
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
            }

            // Power the intake
            // When trigger is held, intake
            else if (gamepad1.left_trigger >= TRIGGER_THRESHOLD) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.REVERSE)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.FORWARD);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // Power the outtake
            // When trigger is held, intake
            else if (gamepad1.left_bumper) {
                // Switch transfer mode to reverse if needed
                if (hardwareController.transfer.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.transfer.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Switch intake mode to reverse if needed
                if (hardwareController.intake.getDirection().equals(DcMotorSimple.Direction.FORWARD)) {
                    hardwareController.intake.setDirection(DcMotorSimple.Direction.REVERSE);
                }
                // Then power intake and transfer
                hardwareController.intake.setPower(HardwareController.INTAKE_POWER);
                hardwareController.transfer.setPower(HardwareController.TRANSFER_POWER);
            }

            // Else don't power either motor
            else {
                hardwareController.intake.setPower(0.0);
                hardwareController.transfer.setPower(0.0);
            }

            hardwareController.autoAimTurret(follower.getPose());
            updateTelemetry();
        }
    }

    public void updateTelemetry() {
        // Locational (On Panels)
        telemetryM.addData("Position (In)", follower.getPose());
        telemetryM.addData("Velocity (In/Sec)", follower.getVelocity());
        telemetryM.addData("Flywheel Velocity (Degrees/Sec)", hardwareController.turretFlywheel.getVelocity(AngleUnit.DEGREES));

        telemetryM.addData("Turret Angle", hardwareController.turretAngle);
        telemetryM.addData("Turret Ticks", hardwareController.turretTicks);
        telemetryM.addData("Tag in Frame", hardwareController.tagDetected);

        telemetryM.addData("Distance: ", hardwareController.distance);

                // Controls (On driver hub telemetry)
        telemetry.addLine("A - Precision Mode\n");

        telemetry.addLine("LT - Intake Power");
        telemetry.addLine("LB - Intake Direction\n");

        telemetry.addLine("RT - Feed");
        telemetry.addLine("RB - Flywheel");

        telemetryM.update();
        telemetry.update();
    }
}
