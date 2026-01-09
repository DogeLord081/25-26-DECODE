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

        // Get joystick inputs - Left Stick: Robot Movement (Strafe/Drive)
        double y = -gamepad1.left_stick_y;  // Forward/backward (pushing stick forward gives negative value)
        double x = gamepad1.left_stick_x;   // Left/right strafe
        // Right Stick: Robot Rotation
        double yaw = gamepad1.right_stick_x; // Rotation (stays robot-centric)

        // Rotate the joystick inputs by the negative of the robot's heading
        // This makes forward on joystick always move the robot away from driver
        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        // Use rotated values for axial and lateral movement
        float axial = (float) rotY;
        float lateral = (float) rotX;

        // Combine the joystick requests for each axis-motion to determine each wheel's power.
        float leftFrontPower = axial + lateral + (float) yaw;
        float rightFrontPower = axial - lateral - (float) yaw;
        float leftBackPower = axial - lateral + (float) yaw;
        float rightBackPower = axial + lateral - (float) yaw;

        // clip the right/left values so that the values never exceed +/- 0.8
        rightFrontPower = (float) Range.clip(rightFrontPower, -0.8, 0.8);
        leftFrontPower = (float) Range.clip(leftFrontPower, -0.8, 0.8);
        leftBackPower = (float) Range.clip(leftBackPower, -0.8, 0.8);
        rightBackPower = (float) Range.clip(rightBackPower, -0.8, 0.8);

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
            } else {
                leftTrapdoor.setPosition(0.1);
            }
        }
        lastGamepad2XState = gamepad2.x;

        // B Button: Right trapdoor toggle
        if (gamepad2.b && !lastGamepad2BState) {
            rightTrapdoorOpen = !rightTrapdoorOpen;
            if (rightTrapdoorOpen) {
                rightTrapdoor.setPosition(0.2);
            } else {
                rightTrapdoor.setPosition(0.1);
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
                leftTrapdoor.setPosition(0.1);
                rightTrapdoor.setPosition(0.1);
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

        // --- Bumpers: Color Selection ---
        // Left Bumper: Select Color: PURPLE
        if (gamepad2.left_bumper && !lastGamepad2LeftBumperState) {
            colorPurpleSelected = true;
            colorGreenSelected = false;
        }
        lastGamepad2LeftBumperState = gamepad2.left_bumper;

        // Right Bumper: Select Color: GREEN
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
            shooter.setPower(0.5);
        } else {
            shooter.setPower(0.0);
        }

        // Right Trigger: Auto shoot
        boolean rightTriggerPressed = gamepad2.right_trigger > 0.5;
        if (rightTriggerPressed && !lastGamepad2RightTriggerState && !shootSequenceActive) {
            startShootSequence();
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

        // Immediate servo actions
        leftTransfer.setPosition(0.0);
        rightTransfer.setPosition(0.5);
        leftTrapdoor.setPosition(0.0);
        rightTrapdoor.setPosition(0.0);
    }

    private void executeShootSequence() {
        // Keep shooter running throughout the sequence
        shooter.setPower(0.5);

        // Intake runs for first 900ms, pauses 900-1000ms, then resumes until end
        if (shootSequenceTimer.milliseconds() < 900) {
            intake.setPower(-1.0);
        } else if (shootSequenceTimer.milliseconds() < 1400) {
            intake.setPower(0.0);
        } else {
            intake.setPower(-1.0);
        }

        // After 500ms delay, set the kicker arm position
        if (shootSequenceTimer.milliseconds() >= 500) {
            leftKickerArm.setPosition(0.5);
        }

        // After 3500ms, set the transfers
        if (shootSequenceTimer.milliseconds() >= 3500) {
            rightTransfer.setPosition(0.0);
            leftTransfer.setPosition(0.5);
        }

        // End the sequence and reset positions (e.g., after 4000ms)
        if (shootSequenceTimer.milliseconds() >= 4500) {
            leftTrapdoor.setPosition(0.1);
            rightTrapdoor.setPosition(0.1);
            leftKickerArm.setPosition(0.0);
            rightTransfer.setPosition(0.5);
            leftTransfer.setPosition(0.0);

            // Turn off shooter and intake at end of sequence
            shooter.setPower(0.0);

            // Reset state variables to match physical state
            leftTrapdoorOpen = false;
            rightTrapdoorOpen = false;
            bothTrapdoorsOpen = false;
            leftKickerArmOpen = false;
            transfersOpen = false;

            shootSequenceActive = false;
        }
    }
}


