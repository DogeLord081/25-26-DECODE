package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static MecanumConstants driveConstants = new MecanumConstants()
            .yVelocity(36.1123)
            .xVelocity(61.0)
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .turnTicksToInches(0.001965)
            .strafeTicksToInches(0.00198)
            .forwardTicksToInches(0.00375)
            .leftPodY(6.102362)
            .rightPodY(-6.102362)
            .strafePodX(1.830709)
            .leftEncoder_HardwareMapName("leftFront")
            .rightEncoder_HardwareMapName("rightBack")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.REVERSE)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD, RevHubOrientationOnRobot.UsbFacingDirection.UP));
    public static FollowerConstants followerConstants = new FollowerConstants()
        .drivePIDFCoefficients(new FilteredPIDFCoefficients(8,0,0,0, 0))
        .headingPIDFSwitch(3)
        .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.118,0,0.169884,0.07))
        .headingPIDFCoefficients(new PIDFCoefficients(0.86,0,0.13068,0.07))
        .translationalPIDFSwitch(3)
        .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.377,0,0.0481,0.066))
        .translationalPIDFCoefficients(new PIDFCoefficients(0.29, 0, 0.03687321, 0.066))
        .lateralZeroPowerAcceleration(-62.7312857143)
        .forwardZeroPowerAcceleration(-29.17948)
        .mass(15.8757)
        .useSecondaryTranslationalPIDF(true)
        .useSecondaryHeadingPIDF(true)
        .useSecondaryDrivePIDF(false);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
