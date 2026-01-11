package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
//import com.qualcomm.robotcore.hardware.CRServo;
import android.graphics.Color;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Tele")
public class Tele extends OpMode {
    protected DcMotor leftFront;
    protected DcMotor rightFront;
    protected DcMotor leftBack;
    protected DcMotor rightBack;
    protected DcMotor shooter;
    protected DcMotor intake;
    protected DcMotor leftLift;
    protected DcMotor rightLift;
    protected Servo leftTrapdoor;
    protected Servo rightTrapdoor;
    protected Servo leftTransfer;
    protected Servo rightTransfer;
    protected Servo leftKickerArm;
    protected Servo rightKickerArm;
    protected Servo leftHoodAdjustment;
    protected Servo rightHoodAdjustment;
    protected ColorSensor colorSensorRight;
    protected ColorSensor colorSensorLeft;
    protected IMU imu;
    protected DistanceSensor distanceSensor;

    // ========== CONTROLLER 1 (DRIVER) STATE ==========
    // Intake toggle state (Right Bumper)
    private boolean intakeToggleOn = false;
    private boolean lastGamepad1RightBumperState = false;

    // Auto-Aim toggle state (Right Trigger)
    private boolean autoAimEnabled = false;
    private boolean lastGamepad1RightTriggerState = false;

    // ========== CONTROLLER 2 (OPERATOR) STATE ==========
    // Trapdoor toggles (Face Buttons)
    private boolean leftTrapdoorOpen = false;
    private boolean rightTrapdoorOpen = false;
    private boolean bothTrapdoorsOpen = false;
    private boolean lastGamepad2XState = false; // Left Trapdoor
    private boolean lastGamepad2BState = false; // Right Trapdoor
    private boolean lastGamepad2YState = false; // Both Trapdoors

    // Transfer toggle (A Button)
    private boolean transfersOpen = false;
    private boolean lastGamepad2AState = false;

    // Kicker Arm states (D-Pad toggles)
    private boolean leftKickerArmOpen = false;
    private boolean rightKickerArmOpen = false;
    private boolean lastDpadLeftState = false;
    private boolean lastDpadRightState = false;

    // Color selection state (Bumpers)
    private boolean colorPurpleSelected = false;
    private boolean colorGreenSelected = false;
    private boolean lastGamepad2LeftBumperState = false;
    private boolean lastGamepad2RightBumperState = false;

    // Shooter speed toggle (Left Trigger)
    private boolean shooterSpeedOn = false;
    private boolean lastGamepad2LeftTriggerState = false;

    // Auto shoot (Right Trigger or Button?) - Image says "Auto shoot". Assuming RT based on position.
    // No toggle needed if it's a sequence trigger, but we need debouncing.
    private boolean lastGamepad2RightTriggerState = false;

    // Timer for shooting sequence
    private ElapsedTime shootSequenceTimer = new ElapsedTime();
    private boolean shootSequenceActive = false;
    private boolean distanceCheckPassed = false; // Tracks if ball was detected at 3000ms
    private boolean kickLeft = false; // Track if left side should be kicked
    private boolean kickRight = false; // Track if right side should be kicked
    private boolean singleBallMode = false; // True if proximity > 6.5 (only one ball)
    private boolean trapdoorsOpenedForSingleBall = false; // Track if trapdoors were opened in single ball mode

    // Debug timing
    private ElapsedTime debugTimer = new ElapsedTime();


    @Override
    public void init() {
        // Initialize drive motors
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // Initialize other motors
        leftLift = hardwareMap.get(DcMotor.class, "leftLift");
        rightLift = hardwareMap.get(DcMotor.class, "rightLift");
        shooter = hardwareMap.get(DcMotor.class, "shooter");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftTransfer = hardwareMap.get(Servo.class, "leftTransfer");
        rightTransfer = hardwareMap.get(Servo.class, "rightTransfer");

        // Initialize servos
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");
        leftKickerArm = hardwareMap.get(Servo.class, "leftKickerArm");
        rightKickerArm = hardwareMap.get(Servo.class, "rightKickerArm");
        leftHoodAdjustment = hardwareMap.get(Servo.class, "leftHoodAdjustment");
        rightHoodAdjustment = hardwareMap.get(Servo.class, "rightHoodAdjustment");

        // Initialize color sensors
        colorSensorRight = hardwareMap.get(ColorSensor.class, "colorSensorRight");
        colorSensorLeft = hardwareMap.get(ColorSensor.class, "colorSensorLeft");

        // Get the distance sensor and motor from hardwareMap
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        // Initialize IMU for field-centric driving
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                RevHubOrientationOnRobot.UsbFacingDirection.UP));
        imu.initialize(parameters);
        imu.resetYaw(); // Reset heading to 0 at start

        // Set motor directions
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        //leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Initialized");

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightLift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {

        // ========== CONTROLLER 1: THE DRIVER (Mobility & Acquisition) ==========

        // Get the robot's heading from the IMU
        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Get joystick inputs
        double y = -gamepad1.left_stick_y;  // Forward/backward (pushing stick forward gives negative value)
        double x = gamepad1.left_stick_x;   // Left/right strafe
        double yaw = gamepad1.right_stick_x; // Rotation (stays robot-centric)

        // Rotate the joystick inputs by the robot's heading
        // This makes forward on joystick always move the robot away from driver (field-centric)
        // rotY = new forward/backward, rotX = new strafe
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Use rotated values for axial and lateral movement
        // rotY is the new forward/backward, rotX is the new strafe
        float lateral = (float) rotX;
        float axial = (float) rotY;

        // Apply a strafe correction factor (strafing is typically less efficient)
        lateral = lateral * 1.1f;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        // clip the right/left values so that the values never exceed +/- 1
        rightFrontPower = (float) Range.clip(rightFrontPower, -0.8, 0.8);
        leftFrontPower = (float) Range.clip(leftFrontPower, -0.8, 0.8);
        leftBackPower = (float) Range.clip(leftBackPower, -0.8, 0.8);
        rightBackPower = (float) Range.clip(rightBackPower, -0.8, 0.8);

        // write the values to the motors
        rightFront.setPower(rightFrontPower);
        leftFront.setPower(leftFrontPower);
        leftBack.setPower(leftBackPower);
        rightBack.setPower(rightBackPower);

        // Right Trigger (Toggle): Auto-Aim / Position - TODO: Implement AprilTag centering
        if (gamepad1.right_trigger > 0.5 && !lastGamepad1RightTriggerState) {
            autoAimEnabled = !autoAimEnabled;
        }
        lastGamepad1RightTriggerState = gamepad1.right_trigger > 0.5;

        // Auto-aim execution placeholder
        if (autoAimEnabled) {
            // autoAimToTarget();
        } else {
            // write the values to the motors (manual control)
            rightFront.setPower(rightFrontPower);
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightBack.setPower(rightBackPower);
        }

        // --- Intake Controls (Controller 1) ---
        // Right Bumper: Intake Toggle (Press once to turn ON, press again to stop)
        if (gamepad1.right_bumper && !lastGamepad1RightBumperState) {
            intakeToggleOn = !intakeToggleOn;
        }
        lastGamepad1RightBumperState = gamepad1.right_bumper;

        // Left Trigger (Hold): Intake IN - overrides toggle
        // Left Bumper: Intake OUT (Reverse/Unjam) - overrides toggle
        if (gamepad1.left_trigger > 0) {
            intake.setPower(-1.0); // Intake in
        } else if (gamepad1.left_bumper) {
            intake.setPower(1.0); // Intake out (reverse/unjam)
        } else if (intakeToggleOn) {
            intake.setPower(-1.0); // Toggle is on, keep intake running
        } else {
            intake.setPower(0.0);
        }

        // ========== CONTROLLER 2: THE OPERATOR (Scoring Logic) ==========

        // --- Face Buttons ---
        // X Button: Left trapdoor toggle
        if (gamepad2.x && !lastGamepad2XState) {
            leftTrapdoorOpen = !leftTrapdoorOpen;
            if (leftTrapdoorOpen) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
            } else {
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2XState = gamepad2.x;

        // B Button: Right trapdoor toggle
        if (gamepad2.b && !lastGamepad2BState) {
            rightTrapdoorOpen = !rightTrapdoorOpen;
            if (rightTrapdoorOpen) {
                rightTrapdoor.setPosition(0.2);
                leftTrapdoor.setPosition(0.2);
            } else {
                rightTrapdoor.setPosition(0.1);
                leftTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2BState = gamepad2.b;

        // Y Button: Open both trapdoors toggle
        if (gamepad2.y && !lastGamepad2YState) {
            bothTrapdoorsOpen = !bothTrapdoorsOpen;
            if (bothTrapdoorsOpen) {
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.2);
                leftTrapdoorOpen = true;
                rightTrapdoorOpen = true;
            } else {
                rightTrapdoor.setPosition(0.1);
                leftTrapdoor.setPosition(0.1);
                leftTrapdoorOpen = false;
                rightTrapdoorOpen = false;
            }
        }
        lastGamepad2YState = gamepad2.y;

        // A Button: Transfer toggle
        if (gamepad2.a && !lastGamepad2AState) {
            transfersOpen = !transfersOpen;
            if (transfersOpen) {
                leftTransfer.setPosition(0.5);
                rightTransfer.setPosition(0.0);
            } else {
                leftTransfer.setPosition(0.0);
                rightTransfer.setPosition(0.5);
            }
        }
        lastGamepad2AState = gamepad2.a;

        // --- D-Pad Controls (Kicker Arms) ---
        // D-Pad Left: Left kicker arm toggle
        if (gamepad2.dpad_left && !lastDpadLeftState) {
            leftKickerArmOpen = !leftKickerArmOpen;
            leftKickerArm.setPosition(leftKickerArmOpen ? 0.5 : 0.0);
        }
        lastDpadLeftState = gamepad2.dpad_left; // Note: using lastDpadLeftState from Tele class fields

        // D-Pad Right: Right kicker arm toggle
        if (gamepad2.dpad_right && !lastDpadRightState) {
            rightKickerArmOpen = !rightKickerArmOpen;
            rightKickerArm.setPosition(rightKickerArmOpen ? 0.075 : 0.5);
        }
        lastDpadRightState = gamepad2.dpad_right;

        // --- Bumpers: Color Selection + Auto Shoot ---
        // Left Bumper: Select Color: PURPLE and start auto shoot
        if (gamepad2.left_bumper && !lastGamepad2LeftBumperState) {
            colorPurpleSelected = true;
            colorGreenSelected = false;
        }
        lastGamepad2LeftBumperState = gamepad2.left_bumper;

        // Right Bumper: Select Color: GREEN and start auto shoot
        if (gamepad2.right_bumper && !lastGamepad2RightBumperState) {
            colorGreenSelected = true;
            colorPurpleSelected = false;
        }
        lastGamepad2RightBumperState = gamepad2.right_bumper;

        // --- Triggers ---
        // Left Trigger: Shooter speed toggle
        boolean leftTriggerPressed = gamepad2.left_trigger > 0.5;
        if (leftTriggerPressed && !lastGamepad2LeftTriggerState) {
            shooterSpeedOn = !shooterSpeedOn;
        }
        lastGamepad2LeftTriggerState = leftTriggerPressed;

        // Set shooter power based on toggle
        if (shooterSpeedOn) {
            shooter.setPower(0.25);
        } else {
            shooter.setPower(0.0);
        }

        // Right Trigger: Backup auto shoot (always left trapdoor/kicker arm, independent of color)
        // Also stops active sequence if pressed during one
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (rightTriggerPressed && !lastGamepad2RightTriggerState) {
            if (shootSequenceActive) {
                stopShootSequence();
            } else {
                startShootSequence();
            }
        }
        lastGamepad2RightTriggerState = rightTriggerPressed;

        if (shootSequenceActive) {
            executeShootSequence();
        }

        // ========== TELEMETRY ==========
        telemetry.addData("--- DRIVER (Gamepad 1) ---", "");
        telemetry.addData("Intake Toggle", intakeToggleOn ? "ON" : "OFF");

        telemetry.addData("--- OPERATOR (Gamepad 2) ---", "");
        telemetry.addData("Left Trapdoor", leftTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Trapdoor", rightTrapdoorOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Both Trapdoors", bothTrapdoorsOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Transfers", transfersOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Left Kicker Arm", leftKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Right Kicker Arm", rightKickerArmOpen ? "OPEN" : "CLOSED");
        telemetry.addData("Shooter Speed", shooterSpeedOn ? "ON" : "OFF");
        telemetry.addData("Color Selected", colorPurpleSelected ? "PURPLE" : (colorGreenSelected ? "GREEN" : "NONE"));

        // Distance Sensor Telemetry
        telemetry.addData("Distance (cm)", "%.2f", distanceSensor.getDistance(DistanceUnit.CM));

        // Color sensor telemetry (HSV)
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
        Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);
        telemetry.addData("Left Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                leftHSV[0], leftHSV[1], leftHSV[2], ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM));
        telemetry.addData("Right Color Sensor (H,S,V,D)", "(%.1f, %.2f, %.2f, %.3f)",
                rightHSV[0], rightHSV[1], rightHSV[2], ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM));
        telemetry.update();
    }

    // ========== PLACEHOLDER FUNCTIONS (TODO: Implement) ==========

    // private void autoAimToTarget() {
    //     // TODO: Use camera/AprilTag to center on target and strafe to correct distance
    // }

    private void startShootSequence() {
        shootSequenceActive = true;
        shootSequenceTimer.reset();

        // Color detection logic
        float[] leftHSV = new float[3];
        float[] rightHSV = new float[3];
        Color.RGBToHSV(colorSensorLeft.red(), colorSensorLeft.green(), colorSensorLeft.blue(), leftHSV);
        Color.RGBToHSV(colorSensorRight.red(), colorSensorRight.green(), colorSensorRight.blue(), rightHSV);

        // Check proximity sensors to determine if single ball mode
        double leftProximity = ((DistanceSensor) colorSensorLeft).getDistance(DistanceUnit.CM);
        double rightProximity = ((DistanceSensor) colorSensorRight).getDistance(DistanceUnit.CM);
        singleBallMode = (leftProximity > 6.5 || rightProximity > 6.5);
        trapdoorsOpenedForSingleBall = false;

        // Determine colors (Purple if hue > 175, otherwise Green)
        boolean leftIsPurple = leftHSV[0] > 175;
        boolean leftIsGreen = !leftIsPurple;
        boolean rightIsPurple = rightHSV[0] > 175;
        boolean rightIsGreen = !rightIsPurple;

        boolean openLeft = false;
        boolean openRight = false;

        // Check against selected color logic
        if (colorPurpleSelected) {
            if (leftIsPurple) openLeft = true;
            if (rightIsPurple) openRight = true;
        } else if (colorGreenSelected) {
            if (leftIsGreen) openLeft = true;
            if (rightIsGreen) openRight = true;
        }

        // Track which kicker arms should activate (only used if not single ball mode)
        // If both balls are the same color, prioritize left
        if (openLeft && openRight) {
            kickLeft = true;
            kickRight = false;
        } else {
            kickLeft = openLeft;
            kickRight = openRight;
        }

        if (singleBallMode) {
            // Single ball mode: stop intake, open both trapdoors
            intake.setPower(0.0);
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        } else {
            // Normal mode: Set trapdoors based on determination using existing button logic patterns
            if (openLeft && openRight) {
                // Both Trapdoors Open logic (matches Y button)
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.2);
            } else if (openLeft) {
                // Left Trapdoor Open logic (matches X button)
                leftTrapdoor.setPosition(0.0);
                rightTrapdoor.setPosition(0.0);
            } else if (openRight) {
                // Right Trapdoor Open logic (matches B button)
                rightTrapdoor.setPosition(0.2);
                leftTrapdoor.setPosition(0.2);
            } else {
                // No match, ensure closed
                leftTrapdoor.setPosition(0.2);
                rightTrapdoor.setPosition(0.0);
            }
        }

        // Immediate servo actions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
    }

    private void executeShootSequence() {
        // Keep shooter running throughout the sequence
        shooter.setPower(0.5);

        if (singleBallMode) {
            // Single ball mode: intake stopped at start, resume after 300ms
            if (shootSequenceTimer.milliseconds() < 300) {
                intake.setPower(0.0);
            } else {
                // After 300ms, resume intake
                if (shootSequenceTimer.milliseconds() < 900 + 300) {
                    intake.setPower(-1.0);
                } else if (shootSequenceTimer.milliseconds() < 1400 + 300) {
                    intake.setPower(0.0);
                } else {
                    intake.setPower(-1.0);
                }
            }
            // Do NOT activate kicker arms in single ball mode
        } else {
            // Normal mode: Intake runs for first 900ms, pauses 900-1400ms, then resumes until end
            if (shootSequenceTimer.milliseconds() < 900) {
                intake.setPower(-1.0);
            } else if (shootSequenceTimer.milliseconds() < 1400) {
                intake.setPower(0.0);
            } else {
                intake.setPower(-1.0);
            }

            // After 500ms delay, set the kicker arm position (only for the side with the ball)
            if (shootSequenceTimer.milliseconds() >= 500) {
                if (kickLeft) {
                    leftKickerArm.setPosition(0.5);
                }
                if (kickRight) {
                    rightKickerArm.setPosition(0.075);
                }
            }
        }

        // At 3000ms, check distance sensor - if > 20cm, ball not in transfer, restart cycle
        if (shootSequenceTimer.milliseconds() >= 3000 && !distanceCheckPassed) {
            double distance = distanceSensor.getDistance(DistanceUnit.CM);
            if (distance > 20) {
                // Ball not detected, restart the cycle
                restartShootSequence();
                return;
            } else {
                // Ball detected, proceed with transfers
                distanceCheckPassed = true;
            }
        }

        // After 3500ms, set the transfers (only if distance check passed)
        if (shootSequenceTimer.milliseconds() >= 3500 && distanceCheckPassed) {
            rightTransfer.setPosition(0.0);
            leftTransfer.setPosition(0.5);
        }

        // End the sequence and reset positions (e.g., after 4000ms)
        if (shootSequenceTimer.milliseconds() >= 4500) {
            rightTrapdoor.setPosition(0.1);
            leftTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightKickerArm.setPosition(0.5);
            rightTransfer.setPosition(0.5);
            leftTransfer.setPosition(0.0);

            // Turn off shooter and intake at end of sequence
            shooter.setPower(0.0);

            // Reset state variables to match physical state
            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            leftKickerArmOpen = false;
            rightKickerArmOpen = false;
            transfersOpen = false;

            shootSequenceActive = false;
            distanceCheckPassed = false;
            kickLeft = false;
            kickRight = false;
            singleBallMode = false;
            trapdoorsOpenedForSingleBall = false;
        }
    }

    private void stopShootSequence() {
        // Stop the sequence and reset all mechanisms
        shootSequenceActive = false;
        distanceCheckPassed = false;
        kickLeft = false;
        kickRight = false;
        singleBallMode = false;
        trapdoorsOpenedForSingleBall = false;

        // Reset servos to closed positions
        leftTrapdoor.setPosition(0.2);
        rightTrapdoor.setPosition(0.0);
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
        rightTransfer.setPosition(0.5);
        leftTransfer.setPosition(0.0);

        // Stop motors
        shooter.setPower(0.0);
        intake.setPower(0.0);

        // Reset state variables to match physical state
        leftTrapdoorOpen = false;
        rightTrapdoorOpen = false;
        bothTrapdoorsOpen = false;
        leftKickerArmOpen = false;
        rightKickerArmOpen = false;
        transfersOpen = false;
    }

    private void restartShootSequence() {
        // Reset the timer to restart the cycle from the beginning
        shootSequenceTimer.reset();
        distanceCheckPassed = false;

        // Do NOT re-read color sensors - keep using the same kickLeft/kickRight values
        // that were originally determined until the ball passes the distance sensor check

        if (singleBallMode) {
            // Single ball mode: open both trapdoors again
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        } else if (kickLeft && kickRight) {
            // Both sides (shouldn't happen since we prioritize left, but handle it)
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.2);
        } else if (kickLeft) {
            // Left side
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        } else if (kickRight) {
            // Right side
            rightTrapdoor.setPosition(0.2);
            leftTrapdoor.setPosition(0.2);
        } else {
            // No match (backup - use left side)
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.0);
        }

        // Reset transfer positions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);

        // Reset kicker arms
        leftKickerArm.setPosition(0.0);
        rightKickerArm.setPosition(0.5);
    }
}


