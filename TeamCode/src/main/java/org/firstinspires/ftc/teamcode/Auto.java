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
    private final Pose intakeBallsPose = new Pose(18.55750819672132, 73.73770491803278, Math.toRadians(180));

    /* Path and PathChain declarations */
    private Path scorePreload;
    private Path afterScanPath;
    private Path joltPath;
    private Path afterjoltPath;
    private Path afterShootPath;
    private Path intakeBallsPath;

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
    }

    /** Main state machine for autonomous path progression **/
    public void autonomousPathUpdate() {
        // Update shooter RPM tracking
        updateShooterRPM();

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
                /* Shooting state - shoot balls in order */
                controlShooterPID();  // Keep controlling shooter

                // *** JOLT LOGIC INSERTION ***
                // If we are on the 3rd ball (index 2) and haven't jolted yet
                if (currentBallIndex == 2 && !thirdBallJoltDone) {
                    // Open both trapdoors immediately (standard for 3rd ball)
                    leftTrapdoor.setPosition(0.0);
                    rightTrapdoor.setPosition(0.2);

                    // Interrupt sequence to perform jolt
                    follower.followPath(joltPath);
                    setPathState(41); // Move to Jolt Out state
                } else {
                    // Normal execution
                    executeShootingSequence();
                }
                break;

            case 41:
                /* JOLT OUT: Wait for robot to move back */
                controlShooterPID();
                if(!follower.isBusy()) {
                    follower.followPath(afterjoltPath);
                    setPathState(42); // Move to Jolt Return state
                }
                break;

            case 42:
                /* JOLT RETURN: Wait for robot to come back */
                controlShooterPID();
                if(!follower.isBusy()) {
                    // Re-engage hold point to keep steady for shooting
                    follower.holdPoint(afterScanPose);

                    // Reset the shoot timer so the intake/kicker sequence restarts fresh
                    shootTimer.reset();

                    // Mark jolt as complete so we don't loop back here
                    thirdBallJoltDone = true;

                    // Return to standard shooting logic
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
                /* Wait for robot to reach afterShootPose */
                if (!follower.isBusy()) {
                    intake.setPower(0);
                    setPathState(-1);
                }
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

    /** Execute the shooting sequence for current ball **/
    private void executeShootingSequence() {
        if (currentBallIndex >= 3) {
            // All balls shot
            setPathState(5);
            return;
        }

        // Determine which side to shoot from based on ball order
        if (!shootSideDecided) {
            char targetColor = ballOrder[currentBallIndex];
            if (currentBallIndex == 2) {
                // Third ball - will use both trapdoors
                currentShootLeft = true;  // Doesn't matter, we'll handle specially
            } else {
                // First two balls - P = left, G = right
                currentShootLeft = (targetColor == 'P');
            }
            shootSideDecided = true;
        }

        double elapsedMs = shootTimer.seconds() * 1000;  // Convert to milliseconds
        boolean isThirdBall = (currentBallIndex == 2);

        // If first ball was pre-loaded, skip directly to shooting phase
        if (currentBallIndex == 0 && firstBallPreloaded && distanceCheckPassed && !shotFired) {
            // First ball is already in transfer, wait for RPM stabilization then shoot
            if (elapsedMs >= 500 && shooterRPM >= TARGET_RPM) {
                // Open transfer to shoot
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        } else if (!shotFired && !(currentBallIndex == 0 && firstBallPreloaded)) {
            // Normal loading sequence for balls 2 and 3 (or if first ball wasn't pre-loaded)

            // Open appropriate trapdoor throughout loading phase
            if (!distanceCheckPassed) {
                if (isThirdBall) {
                    // Third ball - both trapdoors
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

            // Intake timing: on for 0-900ms, pause 900-1400ms, on again after 1400ms
            if (elapsedMs < 900) {
                intake.setPower(-1.0);
            } else if (elapsedMs < 1400) {
                intake.setPower(0.0);
            } else {
                intake.setPower(-1.0);
            }

            // After 500ms delay, activate kicker arm(s)
            if (elapsedMs >= 500) {
                if (isThirdBall) {
                    // Third ball - both kicker arms
                    leftKickerArm.setPosition(0.5);
                    rightKickerArm.setPosition(0.075);
                } else if (currentShootLeft) {
                    leftKickerArm.setPosition(0.5);
                } else {
                    rightKickerArm.setPosition(0.075);
                }
            }

            // At 1500ms, check distance sensor - if ball not detected, restart
            if (elapsedMs >= 1500 && !distanceCheckPassed) {
                double distance = distanceSensor.getDistance(DistanceUnit.CM);
                if (distance > 20) {
                    // Ball not detected, restart the cycle for this ball
                    restartBallSequence(currentShootLeft);
                    return;
                } else {
                    // Ball detected, proceed with transfers
                    distanceCheckPassed = true;
                    // Close trapdoors
                    leftTrapdoor.setPosition(0.1);
                    rightTrapdoor.setPosition(0.1);
                    intake.setPower(0);  // Stop intake
                }
            }

            // After 2000ms and distance check passed, fire if RPM ready
            if (elapsedMs >= 2000 && distanceCheckPassed && !shotFired) {
                if (isRPMReady()) {
                    leftTransfer.setPosition(0.5);
                    rightTransfer.setPosition(0.0);
                    shotFired = true;
                    shotFiredTimer.reset();
                }
            }
        }

        // If distance check passed but shot not yet fired, keep checking RPM
        if (distanceCheckPassed && !shotFired) {
            if (isRPMReady()) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
                shotFired = true;
                shotFiredTimer.reset();
            }
        }

        // After shot is fired, wait 1 second then move to next ball
        if (shotFired && shotFiredTimer.seconds() >= 1.0) {
            // Reset for next ball
            leftTransfer.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);

            // Move to next ball
            currentBallIndex++;
            shotFired = false;
            distanceCheckPassed = false;
            firstBallPreloaded = false;  // Clear the preload flag
            shootSideDecided = false;  // Reset so we decide side for next ball
            thirdBallJoltDone = false; // Reset jolt logic for safety (though not needed if idx > 2)
            shootTimer.reset();
        }
    }

    /** Restart the sequence for the current ball when distance sensor doesn't detect ball **/
    private void restartBallSequence(boolean shootLeft) {
        // Reset the timer to restart the cycle from the beginning
        shootTimer.reset();
        distanceCheckPassed = false;
        shotFired = false;

        // Re-open the appropriate trapdoor
        if (shootLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        }

        // Reset transfer positions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }

    /** Load the first ball into the transfer while turning (don't wait for RPM, just get ball ready) **/
    private void executeFirstBallLoading() {
        // Only run this for the first ball and if we haven't already passed the distance check
        if (currentBallIndex != 0 || distanceCheckPassed) {
            return;
        }

        // Determine which side based on ball order - P = left, G = right
        if (!shootSideDecided) {
            char targetColor = ballOrder[0];
            currentShootLeft = (targetColor == 'P');
            shootSideDecided = true;
        }

        double elapsedMs = shootTimer.seconds() * 1000;  // Convert to milliseconds

        // Open appropriate trapdoor throughout loading phase
        if (currentShootLeft) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else {
            leftTrapdoor.setPosition(0.2);
            rightTrapdoor.setPosition(0.2);
        }

        // Intake timing: on for 0-900ms, pause 900-1400ms, on again after 1400ms
        if (elapsedMs < 900) {
            intake.setPower(-1.0);
        } else if (elapsedMs < 1400) {
            intake.setPower(0.0);
        } else {
            intake.setPower(-1.0);
        }

        // After 500ms delay, activate kicker arm
        if (elapsedMs >= 500) {
            if (currentShootLeft) {
                leftKickerArm.setPosition(0.5);
            } else {
                rightKickerArm.setPosition(0.075);
            }
        }

        // At 1500ms, check distance sensor - if ball not detected, restart
        if (elapsedMs >= 1500) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance > 20) {
                // Ball not detected, restart the loading
                restartBallSequence(currentShootLeft);
            } else {
                // Ball detected in transfer - ready to shoot when RPM is up
                distanceCheckPassed = true;
                // Close trapdoors
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
                intake.setPower(0);  // Stop intake
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
        if (pathState == 4 || pathState == 41 || pathState == 42) {
            telemetry.addData("--- SHOOTING ---", "");
            telemetry.addData("Current Ball", currentBallIndex + 1);
            telemetry.addData("Target Color", currentBallIndex < 3 ? (ballOrder[currentBallIndex] == 'P' ? "PURPLE" : "GREEN") : "Done");
            telemetry.addData("Jolting", (pathState == 41 || pathState == 42) ? "YES" : "NO");

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