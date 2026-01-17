package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import android.graphics.Color;

@Autonomous(name = "DECODE 25-26 Auto")
public class Auto extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    private HuskyLens huskyLens;

    // Hardware for shooting
    private DcMotor shooter;
    private DcMotor intake;
    private Servo leftTrapdoor;
    private Servo rightTrapdoor;
    private Servo leftTransfer;
    private Servo rightTransfer;
    private Servo leftKickerArm;
    private Servo rightKickerArm;
    private Servo leftHoodAdjustment;
    private Servo rightHoodAdjustment;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensorLeft;
    private ColorSensor colorSensorRight;

    // Shooter constants
    private static final double TARGET_RPM = 1900.0;
    private static final double RPM_TOLERANCE_PERCENT = 0.05;  // 5% tolerance
    private static final double SHOOTER_TICKS_PER_REV = 28.0;

    // Shooter state tracking
    private double shooterRPM = 0.0;
    private int lastShooterEncoderPosition = 0;
    private ElapsedTime velocityTimer = new ElapsedTime();
    private ElapsedTime shootTimer = new ElapsedTime();

    // Shooting sequence state
    private int currentBallIndex = 0;  // 0, 1, 2 for the three balls
    private boolean shotFired = false;
    private boolean distanceCheckPassed = false;
    private boolean firstBallPreloaded = false;  // Track if first ball was loaded during turning
    private ElapsedTime shotFiredTimer = new ElapsedTime();  // Track time since shot was fired
    private boolean currentShootLeft = false;  // Cached value for which side to shoot from
    private boolean shootSideDecided = false;  // Track if we've decided which side to shoot from

    // *** NEW VARIABLE FOR JOLT LOGIC ***
    private boolean thirdBallJoltDone = false;

    // Ball order based on AprilTag (P = Purple/Left, G = Green/Right)
    // ID 1: PPG, ID 2: PGP, ID 3: GPP
    private char[] ballOrder = new char[3];

    /* Define poses for the autonomous routine */
    private final Pose startPose = new Pose(57.328, 134.590, Math.toRadians(270));
    private final Pose scorePose = new Pose(52.328, 115.18032786885244, Math.toRadians(250));
    private final Pose afterScanPose = new Pose(52.328, 100.18032786885244, Math.toRadians(325));
    private final Pose joltPose = new Pose(55.328, 97.18032786885244, Math.toRadians(325));
    private final Pose afterShootPose = new Pose(41.55750819672132, 73.73770491803278, Math.toRadians(180));
    private final Pose intakeBallsPose = new Pose(6.55750819672132, 73.73770491803278, Math.toRadians(180));

    /* Path and PathChain declarations */
    private Path scorePreload;
    private Path afterScanPath;
    private Path joltPath;
    private Path afterjoltPath;
    private Path afterShootPath;
    private Path intakeBallsPath;

    // New Return Paths
    private Path returnToAfterShootPath;
    private Path returnToScanPath;

    /* AprilTag scanning state */
    private int detectedAprilTagId = -1;  // -1 means not yet detected

    /** Build all paths for the autonomous routine **/
    public void buildPaths() {
        /* This is our scorePreload path using a BezierLine (straight line) */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        /* Path after scanning AprilTag */
        afterScanPath = new Path(new BezierLine(scorePose, afterScanPose));
        afterScanPath.setLinearHeadingInterpolation(scorePose.getHeading(), afterScanPose.getHeading());

        joltPath = new Path(new BezierLine(afterScanPose, joltPose));
        joltPath.setLinearHeadingInterpolation(afterScanPose.getHeading(), joltPose.getHeading());

        afterjoltPath = new Path(new BezierLine(joltPose, afterScanPose));
        afterjoltPath.setLinearHeadingInterpolation(joltPose.getHeading(), afterScanPose.getHeading());

        /* Path after shooting all balls */
        afterShootPath = new Path(new BezierLine(afterScanPose, afterShootPose));
        afterShootPath.setLinearHeadingInterpolation(afterScanPose.getHeading(), afterShootPose.getHeading());

        intakeBallsPath = new Path(new BezierLine(afterShootPose, intakeBallsPose));
        intakeBallsPath.setLinearHeadingInterpolation(afterShootPose.getHeading(), intakeBallsPose.getHeading());

        // Return trip paths
        returnToAfterShootPath = new Path(new BezierLine(intakeBallsPose, afterShootPose));
        returnToAfterShootPath.setLinearHeadingInterpolation(intakeBallsPose.getHeading(), afterShootPose.getHeading());

        returnToScanPath = new Path(new BezierLine(afterShootPose, afterScanPose));
        returnToScanPath.setLinearHeadingInterpolation(afterShootPose.getHeading(), afterScanPose.getHeading());
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

        // *** EMERGENCY PARK LOGIC ***
        // If we have crossed 27 seconds and aren't already parking, abort and move to park
        // We check pathState != 99 and != 100 to ensure we don't re-trigger this once started
        if (opmodeTimer.getElapsedTimeSeconds() > 29 && pathState != 99 && pathState != 100) {
            // Shut down mechanisms
            shooter.setPower(0);
            intake.setPower(0);

            // Create a dynamic path from WHEREVER we are right now to the parking spot
            // This ensures we don't try to jump to the start of a pre-recorded path
            Pose currentPose = follower.getPose();
            Path emergencyParkPath = new Path(new BezierLine(currentPose, afterShootPose));
            emergencyParkPath.setLinearHeadingInterpolation(currentPose.getHeading(), afterShootPose.getHeading());

            // Follow it immediately
            follower.followPath(emergencyParkPath);
            setPathState(99);
            return; // Exit the function so we don't execute other cases below
        }

        switch (pathState) {
            case 0:
                // Start shooter spinning to target RPM
                shooter.setPower(0.5);  // Initial power to start spinning up
                follower.followPath(scorePreload);
                setPathState(1);
                break;
            case 1:
                /* Wait until the robot has finished following the path */
                controlShooterPID();  // Keep controlling shooter
                if (!follower.isBusy()) {
                    /* Move to state 2 to scan for AprilTag */
                    setPathState(2);
                }
                break;
            case 2:
                /* Wait for AprilTag to be detected */
                controlShooterPID();  // Keep controlling shooter
                HuskyLens.Block[] blocks = huskyLens.blocks();
                if (blocks.length > 0 && detectedAprilTagId == -1) {
                    /* Freeze the detected AprilTag ID */
                    detectedAprilTagId = blocks[0].id;

                    /* Set ball order based on AprilTag ID */
                    switch (detectedAprilTagId) {
                        case 1:
                            ballOrder = new char[]{'P', 'P', 'G'};
                            break;
                        case 2:
                            ballOrder = new char[]{'P', 'G', 'P'};
                            break;
                        case 3:
                            ballOrder = new char[]{'G', 'P', 'P'};
                            break;
                        default:
                            ballOrder = new char[]{'P', 'P', 'G'};  // Default to ID 1 pattern
                            break;
                    }

                    /* Turn in place to afterScanPose heading using holdPoint */
                    follower.holdPoint(afterScanPose);

                    /* Start loading first ball immediately to save time */
                    currentBallIndex = 0;
                    shootTimer.reset();
                    shotFired = false;
                    distanceCheckPassed = false;
                    thirdBallJoltDone = false; // Reset jolt flag

                    setPathState(3);
                }
                break;
            case 3:
                /* Wait for the robot to finish turning while also loading first ball */
                controlShooterPID();  // Keep controlling shooter

                // Run the first ball loading sequence in parallel with turning
                executeFirstBallLoading();

                if (pathTimer.getElapsedTimeSeconds() > 4.0) {
                    /* Turn complete, continue to shooting state */
                    // If first ball was pre-loaded, mark it so we skip loading phase
                    if (distanceCheckPassed) {
                        firstBallPreloaded = true;
                    }
                    // Reset shoot timer for the shooting phase
                    shootTimer.reset();
                    setPathState(4);
                }
                break;
            case 4:
                /* ROUND 1: Shooting state - shoot balls in order (Assumption based) */
                controlShooterPID();

                // JOLT LOGIC
                if (currentBallIndex == 2 && !thirdBallJoltDone) {
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.2);
                    follower.followPath(joltPath);
                    setPathState(41);
                } else {
                    executeShootingSequence(false); // False = Don't use sensors, use fixed assumption
                }
                break;

            case 41: // Jolt Out (Round 1)
                controlShooterPID();
                if(!follower.isBusy()) {
                    follower.followPath(afterjoltPath);
                    setPathState(42);
                }
                break;

            case 42: // Jolt Return (Round 1)
                controlShooterPID();
                if(!follower.isBusy()) {
                    follower.holdPoint(afterScanPose);
                    shootTimer.reset();
                    thirdBallJoltDone = true;
                    setPathState(4);
                }
                break;

            case 5:
                /* All balls shot, go to afterShootPose */
                shooter.setPower(0);
                intake.setPower(0);
                follower.followPath(afterShootPath);
                setPathState(6);
                break;
            case 6:
                /* Wait for robot to reach afterShootPose */
                if (!follower.isBusy()) {
                    intake.setPower(-1.0);
                    follower.followPath(intakeBallsPath);
                    setPathState(7);
                }
                break;
            case 7:
                /* Wait for robot to reach intakeBallsPose */
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    // Start return trip
                    follower.followPath(returnToAfterShootPath);
                    setPathState(8);
                }
                break;

            // *** NEW RETURN AND SECOND SHOOTING LOGIC ***

            case 8:
                /* Arrived at afterShootPose, go to Scan Pose */
                if(!follower.isBusy()){
                    shooter.setPower(0.5); // Spin up shooter again
                    follower.followPath(returnToScanPath);
                    setPathState(9);
                }
                break;

            case 9:
                /* Arrived back at Shooting Position */
                controlShooterPID();
                if(!follower.isBusy()){
                    follower.holdPoint(afterScanPose);

                    // Reset variables for Round 2
                    currentBallIndex = 0;
                    shotFired = false;
                    distanceCheckPassed = false;
                    thirdBallJoltDone = false;
                    shootSideDecided = false;
                    firstBallPreloaded = false; // Not relevant for round 2
                    shootTimer.reset();

                    setPathState(10);
                }
                break;

            case 10:
                /* ROUND 2: Shooting with Color Sensors */
                controlShooterPID();

                // JOLT LOGIC FOR ROUND 2
                if (currentBallIndex == 2 && !thirdBallJoltDone) {
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.2);
                    follower.followPath(joltPath);
                    setPathState(101); // Go to Jolt Out Round 2
                } else {
                    // True = Use sensors to find the correct ball
                    executeShootingSequence(true);
                }

                // Check if all balls done for Round 2
                if(currentBallIndex >= 3) {
                    setPathState(11); // Done
                }
                break;

            case 101: // Jolt Out (Round 2)
                controlShooterPID();
                if(!follower.isBusy()) {
                    follower.followPath(afterjoltPath);
                    setPathState(102);
                }
                break;

            case 102: // Jolt Return (Round 2)
                controlShooterPID();
                if(!follower.isBusy()) {
                    follower.holdPoint(afterScanPose);
                    shootTimer.reset();
                    thirdBallJoltDone = true;
                    setPathState(10);
                }
                break;

            case 11:
                // Final state - Park or Stop
                shooter.setPower(0);
                intake.setPower(0);
                setPathState(-1);
                break;

            // *** EMERGENCY PARK STATE ***
            case 99:
                // Waiting for the robot to reach parking spot
                if(!follower.isBusy()) {
                    setPathState(100);
                }
                break;
            case 100:
                // Robot parked, do nothing
                break;
        }
    }

    /** Update shooter RPM from encoder **/
    private void updateShooterRPM() {
        int currentShooterPosition = shooter.getCurrentPosition();
        double deltaTime = velocityTimer.seconds();

        if (deltaTime > 0.02) {  // Update velocity every 20ms
            int deltaTicks = currentShooterPosition - lastShooterEncoderPosition;
            double ticksPerSecond = Math.abs(deltaTicks / deltaTime);
            shooterRPM = (ticksPerSecond / SHOOTER_TICKS_PER_REV) * 60.0;
            lastShooterEncoderPosition = currentShooterPosition;
            velocityTimer.reset();
        }
    }

    /** Simple proportional control for shooter RPM **/
    private void controlShooterPID() {
        double error = TARGET_RPM - shooterRPM;
        double kP = 0.0003;
        double kF = 1.0 / 4900.0;  // Feedforward based on max RPM

        double power = (TARGET_RPM * kF) + (error * kP);
        power = Math.max(0.0, Math.min(1.0, power));
        shooter.setPower(power);
    }

    /** Check if RPM is within tolerance **/
    private boolean isRPMReady() {
        double rpmLowerBound = TARGET_RPM * (1.0 - RPM_TOLERANCE_PERCENT);
        double rpmUpperBound = TARGET_RPM * (1.0 + RPM_TOLERANCE_PERCENT);
        return shooterRPM >= rpmLowerBound && shooterRPM <= rpmUpperBound;
    }

    /** Execute the shooting sequence for current ball
     * @param useSensors - If true, uses color sensors to determine which side to shoot.
     * If false, assumes balls are in the pre-loaded spots.
     **/
    private void executeShootingSequence(boolean useSensors) {
        if (currentBallIndex >= 3) {
            // All balls shot
            // Case switching is handled in the main loop for Round 2,
            // but for Round 1 we might need to trigger state change here.
            if (!useSensors) setPathState(5);
            return;
        }

        // Determine which side to shoot from
        if (!shootSideDecided) {
            char targetColor = ballOrder[currentBallIndex];

            if (currentBallIndex == 2) {
                // Third ball - will use both trapdoors regardless
                currentShootLeft = true;
            } else if (useSensors) {
                // *** COLOR SENSOR LOGIC ENABLED ***
                float[] hsvLeft = new float[3];
                Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), hsvLeft);
                // Simple threshold: > 175 is typically Purple/Blue-ish, < 150 is Green/Yellow-ish
                boolean leftIsPurple = hsvLeft[0] > 175;

                if (targetColor == 'P') {
                    // We need Purple. If Left is Purple, shoot Left. Else shoot Right.
                    currentShootLeft = leftIsPurple;
                } else {
                    // We need Green. If Left is NOT Purple (Green), shoot Left. Else shoot Right.
                    currentShootLeft = !leftIsPurple;
                }
            } else {
                // Standard Round 1 Logic (Assumption based on AprilTag pattern)
                currentShootLeft = (targetColor == 'P');
            }
            shootSideDecided = true;
        }

        double elapsedMs = shootTimer.seconds() * 1000;  // Convert to milliseconds
        boolean isThirdBall = (currentBallIndex == 2);

        // Pre-load logic only applies to Round 1 (useSensors == false)
        if (!useSensors && currentBallIndex == 0 && firstBallPreloaded && distanceCheckPassed && !shotFired) {
            if (elapsedMs >= 500 && shooterRPM >= TARGET_RPM) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        } else if (!shotFired) {
            // Normal loading sequence (Used for Round 1 balls 2&3, and ALL Round 2 balls)

            // Open appropriate trapdoor throughout loading phase
            if (!distanceCheckPassed) {
                if (isThirdBall) {
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.2);
                } else if (currentShootLeft) {
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.0);
                } else {
                    leftTrapdoor.setPosition(0.2);
                    rightTrapdoor.setPosition(0.2);
                }
            }

            // Intake pulsing
            if (elapsedMs < 900) {
                intake.setPower(-1.0);
            } else if (elapsedMs < 1400) {
                intake.setPower(0.0);
            } else {
                intake.setPower(-1.0);
            }

            // Kicker arm activation
            if (elapsedMs >= 500) {
                if (isThirdBall) {
                    leftKickerArm.setPosition(0.5);
                    rightKickerArm.setPosition(0.075);
                } else if (currentShootLeft) {
                    leftKickerArm.setPosition(0.5);
                } else {
                    rightKickerArm.setPosition(0.075);
                }
            }

            // Distance Check
            if (elapsedMs >= 1500 && !distanceCheckPassed) {
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                if (distance > 20) {
                    restartBallSequence(currentShootLeft);
                    return;
                } else {
                    distanceCheckPassed = true;
                    leftTrapdoor.setPosition(0.1);
                    rightTrapdoor.setPosition(0.1);
                    intake.setPower(0);
                }
            }

            // Fire if ready
            if (elapsedMs >= 2000 && distanceCheckPassed && !shotFired) {
                if (isRPMReady()) {
                    leftTransfer.setPosition(0.5);
                    rightTransfer.setPosition(0.0);
                    shotFired = true;
                    shotFiredTimer.reset();
                }
            }
        }

        // Keep checking RPM if waiting to fire
        if (distanceCheckPassed && !shotFired) {
            if (isRPMReady()) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // Reset after shot
        if (shotFired && shotFiredTimer.seconds() >= 1.0) {
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);

            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            firstBallPreloaded = false;
            shootSideDecided = false;
            thirdBallJoltDone = false; // Reset for safety (though logic prevents reuse in same round)
            shootTimer.reset();
        }
    }

    /** Restart the sequence for the current ball when distance sensor doesn't detect ball **/
    private void restartBallSequence(boolean shootLeft) {
        shootTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;

        if (shootLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        }

        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /** Load the first ball into the transfer while turning (don't wait for RPM, just get ball ready) **/
    private void executeFirstBallLoading() {
        if (currentBallIndex != 0 || distanceCheckPassed) {
            return;
        }

        if (!shootSideDecided) {
            char targetColor = ballOrder[0];
            currentShootLeft = (targetColor == 'P');
            shootSideDecided = true;
        }

        double elapsedMs = shootTimer.seconds() * 1000;

        if (currentShootLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        }

        if (elapsedMs < 900) {
            intake.setPower(-1.0);
        } else if (elapsedMs < 1400) {
            intake.setPower(0.0);
        } else {
            intake.setPower(-1.0);
        }

        if (elapsedMs >= 500) {
            if (currentShootLeft) {
                leftKickerArm.setPosition(0.5);
            } else {
                rightKickerArm.setPosition(0.075);
            }
        }

        if (elapsedMs >= 1500) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance > 20) {
                restartBallSequence(currentShootLeft);
            } else {
                distanceCheckPassed = true;
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                intake.setPower(0);
            }
        }
    }

    /** Change the path state and reset the timer **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        // These loop the movements of the robot
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.addData("Time Remaining", String.format("%.1f s", 30.0 - opmodeTimer.getElapsedTimeSeconds()));

        // HuskyLens AprilTag detection
        HuskyLens.Block[] blocks = huskyLens.blocks();
        if (detectedAprilTagId != -1) {
            // Show the frozen/locked AprilTag ID
            telemetry.addData("AprilTag ID (LOCKED)", detectedAprilTagId);
            telemetry.addData("Ball Order", "" + ballOrder[0] + ballOrder[1] + ballOrder[2]);
        } else if (blocks.length > 0) {
            telemetry.addData("AprilTag ID", blocks[0].id);
            telemetry.addData("AprilTag X", blocks[0].x);
            telemetry.addData("AprilTag Y", blocks[0].y);
        } else {
            telemetry.addData("AprilTag", "None detected");
        }

        // Shooter status
        telemetry.addData("--- SHOOTER ---", "");
        telemetry.addData("Target RPM", TARGET_RPM);
        telemetry.addData("Current RPM", "%.0f", shooterRPM);
        telemetry.addData("RPM Ready", isRPMReady() ? "YES" : "NO");

        // Shooting sequence status
        if (pathState == 4 || pathState == 41 || pathState == 42 || pathState == 10 || pathState == 101 || pathState == 102) {
            telemetry.addData("--- SHOOTING ---", "");
            telemetry.addData("Round", (pathState >= 10) ? "2 (SENSORS)" : "1 (ASSUMED)");
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");
            telemetry.addData("Jolting", (pathState == 41 || pathState == 42 || pathState == 101 || pathState == 102) ? "YES" : "NO");

            // Show color sensor readings
            float[] leftHSV = new float[3];
            float[] rightHSV = new float[3];
            Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
            Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

            // Show proximity readings
            double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
            double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);

            telemetry.addData("Left Sensor", (leftHSV[0] > 175 ? "PURPLE" : "GREEN") +
                    " (prox: " + String.format("%.1f", leftProximity) + "cm" +
                    (leftProximity < 6.5 ? " BALL" : "") + ")");
            telemetry.addData("Right Sensor", (rightHSV[0] > 175 ? "PURPLE" : "GREEN") +
                    " (prox: " + String.format("%.1f", rightProximity) + "cm" +
                    (rightProximity < 6.5 ? " BALL" : "") + ")");

            // Show transfer distance sensor
            double transferDistance = distanceSensor.getDistance(DistanceUnit.CM);
            telemetry.addData("Transfer Distance", String.format("%.1f", transferDistance) + "cm" +
                    (transferDistance < 20 ? " BALL DETECTED" : " NO BALL - WILL RETRY"));

            telemetry.addData("Distance Check", distanceCheckPassed ? "PASSED" : "WAITING");
            telemetry.addData("Shot Fired", shotFired ? "YES" : "NO");
            telemetry.addData("Shoot Timer", String.format("%.2f", shootTimer.seconds()) + "s");
        } else if (pathState == 99 || pathState == 100) {
            telemetry.addData("--- EMERGENCY PARK ---", "MOVING TO PARK");
        }

        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        // Initialize HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskyLens");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Problem communicating with HuskyLens");
        } else {
            telemetry.addData("HuskyLens", "Connected");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // Initialize shooter hardware
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        shooter.setDirection(DcMotor.Direction.REVERSE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize servos
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize distance sensor
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize color sensors
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");

        // Set initial servo positions (closed/neutral)
        leftTrapdoor.setPosition(0.1);
        rightTrapdoor.setPosition(0.1);
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);

        // Set hood position for shooting
        leftHoodAdjustment.setPosition(0.3);
        rightHoodAdjustment.setPosition(0.0);  // Inverse relationship

        // Initialize velocity timer
        velocityTimer.reset();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode. **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}
}