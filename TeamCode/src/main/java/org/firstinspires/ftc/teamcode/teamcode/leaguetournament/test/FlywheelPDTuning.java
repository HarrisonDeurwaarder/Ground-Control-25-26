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

package org.firstinspires.ftc.teamcode.teamcode.leaguetournament.test;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.graph.PanelsGraph;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.teamcode.leaguetournament.HardwareController;

@Configurable
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="Flywheel PIDF Tuner", group="Test")
public class FlywheelPDTuning extends LinearOpMode {

    private TelemetryManager telemetryM;
    private GraphManager graphM;
    private HardwareController hardwareController;

    public static double targetSpeed = 28 * 30; // ticks/sec
    public static double p = 0.0;
    public static double i = 3.0;
    public static double d = 3.0;
    public static double f = 3.0;

    @Override
    public void runOpMode() {

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        hardwareController = new HardwareController(hardwareMap);

        waitForStart();

        /* ###############################
                        START
           ############################### */

        // Functional loop of OpMode
        while (opModeIsActive()) {
            // Set target speed
            hardwareController.turretFlywheel.setVelocityPIDFCoefficients(p, i, d, f);
            hardwareController.turretFlywheel.setVelocity(targetSpeed);
            double error = targetSpeed - hardwareController.turretFlywheel.getVelocity();
            // Panels telemetry
            telemetryM.addData("Target Speed (RPS)", targetSpeed / 28);
            telemetryM.addData("Current Speed (RPS)", hardwareController.turretFlywheel.getVelocity() / 28);
            telemetryM.addData("Error", Math.round(10 * error) / 280);
            telemetryM.addData("Power", Math.round(10 * hardwareController.turretFlywheel.getPower()) / 10);

            graphM.addData("Target Speed", targetSpeed);
            graphM.addData("Current Speed", hardwareController.turretFlywheel.getVelocity());

            telemetryM.update();
            graphM.update();
        }
    }
}