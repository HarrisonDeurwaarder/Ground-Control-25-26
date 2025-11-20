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

package org.firstinspires.ftc.teamcode.teamcode.leaguemeet0;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode runs a drivetrain forward for one second
 */

@Autonomous(name="League Meet 0 (Auto)", group="League Meet 0")
public class AutoLeagueMeet0 extends LinearOpMode {

    static final double DRIVE_SPEED = 0.5;

    // Drive duration in seconds
    static final double DURATION = 3.0;

    /* Declare OpMode members. */
    private DcMotor frontLeftDrive = null;
    private DcMotor frontRightDrive = null;

    private DcMotor backLeftDrive = null;
    private DcMotor backRightDrive = null;

    private ElapsedTime runtime = new ElapsedTime();


    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_fl");
        frontRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_fr");
        backLeftDrive = hardwareMap.get(DcMotor.class, "drivetrain_bl");
        backRightDrive = hardwareMap.get(DcMotor.class, "drivetrain_br");

        // The left-side motors need to be reversed
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);

        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // reset the timeout time and start motion.
            runtime.reset();

            frontLeftDrive.setPower(DRIVE_SPEED);
            backLeftDrive.setPower(DRIVE_SPEED);

            frontRightDrive.setPower(DRIVE_SPEED);
            backRightDrive.setPower(DRIVE_SPEED);

            // Keep looping while the OpMode is active
            // AND the robot has yet to drive for its duration
            // Track remaining time
            while (opModeIsActive() &&
                    (runtime.seconds() < DURATION)) {

                // Display remaining drive time
                telemetry.addData("Active Time", "%.1f seconds", runtime.seconds());
                telemetry.addData("Remaining Drive Time", "%.1f seconds", DURATION - runtime.seconds());
                telemetry.addData("Front Left : Right", "%4.2f : %4.2f", DRIVE_SPEED, DRIVE_SPEED);
                telemetry.addData("Back  Left : Right", "%4.2f : %4.2f", DRIVE_SPEED, DRIVE_SPEED);
                telemetry.update();
            }

            // Stop all motion
            frontLeftDrive.setPower(0.0);
            backLeftDrive.setPower(0.0);

            frontRightDrive.setPower(0.0);
            backRightDrive.setPower(0.0);
        }
    }
}