package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class ConstantsAlpha {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(20.0)
            // Deceleration rates
            .forwardZeroPowerAcceleration(-30.0)
            .lateralZeroPowerAcceleration(-70.0)
            // Enable secondary PID
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            // PIDs
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.2, 0.0, 0.02, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0.0, 0.01, 0.6, 0.0));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            // Drivetrain names
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            // Default drivetrain directions
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            // Max velocities
            .xVelocity(100.0)
            .yVelocity(100.0);

    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("sensor_otos")
            // Units for the OTOS
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            // Measurement conversions
            .linearScalar(1.03)
            .angularScalar(1.00)
            // OTOS offset from center of robot
            .offset(new SparkFunOTOS.Pose2D(4.25, 0.0, -Math.PI / 2.0));

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .OTOSLocalizer(localizerConstants)
                .build();
    }
}
