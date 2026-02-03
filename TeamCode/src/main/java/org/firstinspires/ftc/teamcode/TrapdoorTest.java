package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Test OpMode for trapdoor servo positions.
 *
 * Trapdoor positions from Tele.java:
 * - Left Trapdoor:  Closed = 0.1, Open = 0.0
 * - Right Trapdoor: Closed = 0.1, Open = 0.2
 *
 * Controls:
 * - X: OPEN LEFT trapdoor
 * - Y: CLOSE LEFT trapdoor
 * - B: OPEN RIGHT trapdoor
 * - A: CLOSE RIGHT trapdoor
 * - Left Bumper: OPEN BOTH trapdoors
 * - Right Bumper: CLOSE BOTH trapdoors
 * - D-Pad Up/Down: Fine-tune LEFT trapdoor position
 * - D-Pad Left/Right: Fine-tune RIGHT trapdoor position
 * - Left Trigger: Decrease adjustment step size
 * - Right Trigger: Increase adjustment step size
 */
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TrapdoorTest", group = "Test")
public class TrapdoorTest extends OpMode {
    protected Servo leftTrapdoor;
    protected Servo rightTrapdoor;

    // Positions from Tele.java
    private static final double LEFT_CLOSED = 0.1;
    private static final double LEFT_OPEN = 0.2;
    private static final double RIGHT_CLOSED = 0.1;
    private static final double RIGHT_OPEN = 0.0;

    // Current positions for fine-tuning
    private double leftPosition = LEFT_CLOSED;
    private double rightPosition = RIGHT_CLOSED;

    // Step size for fine-tuning
    private double stepSize = 0.01;

    // State tracking for D-pad fine-tuning (debouncing)
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;

    // Track open/closed state for telemetry display
    private boolean leftOpen = false;
    private boolean rightOpen = false;

    @Override
    public void init() {
        // Initialize servos
        leftTrapdoor = hardwareMap.get(Servo.class, "leftTrapdoor");
        rightTrapdoor = hardwareMap.get(Servo.class, "rightTrapdoor");

        // Set initial positions (closed)
        leftTrapdoor.setPosition(LEFT_CLOSED);
        rightTrapdoor.setPosition(RIGHT_CLOSED);
        leftPosition = LEFT_CLOSED;
        rightPosition = RIGHT_CLOSED;

        telemetry.addData("Status", "Initialized");
        telemetry.addData("Controls", "X=Left, B=Right, Y=Both Open, A=Both Closed");
        telemetry.addData("Fine-tune", "DPad Up/Down=Left, DPad Left/Right=Right");
        telemetry.update();
    }

    @Override
    public void loop() {
        // === Direct Button Controls ===

        // X Button: OPEN LEFT trapdoor
        if (gamepad1.x) {
            leftOpen = true;
            leftPosition = LEFT_OPEN;
            leftTrapdoor.setPosition(leftPosition);
        }

        // Y Button: CLOSE LEFT trapdoor
        if (gamepad1.y) {
            leftOpen = false;
            leftPosition = LEFT_CLOSED;
            leftTrapdoor.setPosition(leftPosition);
        }

        // B Button: OPEN RIGHT trapdoor
        if (gamepad1.b) {
            rightOpen = true;
            rightPosition = RIGHT_OPEN;
            rightTrapdoor.setPosition(rightPosition);
        }

        // A Button: CLOSE RIGHT trapdoor
        if (gamepad1.a) {
            rightOpen = false;
            rightPosition = RIGHT_CLOSED;
            rightTrapdoor.setPosition(rightPosition);
        }

        // Left Bumper: OPEN BOTH trapdoors
        if (gamepad1.left_bumper) {
            leftOpen = true;
            rightOpen = true;
            leftPosition = LEFT_OPEN;
            rightPosition = RIGHT_OPEN;
            leftTrapdoor.setPosition(leftPosition);
            rightTrapdoor.setPosition(rightPosition);
        }

        // Right Bumper: CLOSE BOTH trapdoors
        if (gamepad1.right_bumper) {
            leftOpen = false;
            rightOpen = false;
            leftPosition = LEFT_CLOSED;
            rightPosition = RIGHT_CLOSED;
            leftTrapdoor.setPosition(leftPosition);
            rightTrapdoor.setPosition(rightPosition);
        }

        // === Fine-Tune Controls ===

        // D-Pad Up: Increase LEFT trapdoor position
        if (gamepad1.dpad_up && !lastDpadUp) {
            leftPosition = Math.min(1.0, leftPosition + stepSize);
            leftTrapdoor.setPosition(leftPosition);
        }
        lastDpadUp = gamepad1.dpad_up;

        // D-Pad Down: Decrease LEFT trapdoor position
        if (gamepad1.dpad_down && !lastDpadDown) {
            leftPosition = Math.max(0.0, leftPosition - stepSize);
            leftTrapdoor.setPosition(leftPosition);
        }
        lastDpadDown = gamepad1.dpad_down;

        // D-Pad Right: Increase RIGHT trapdoor position
        if (gamepad1.dpad_right && !lastDpadRight) {
            rightPosition = Math.min(1.0, rightPosition + stepSize);
            rightTrapdoor.setPosition(rightPosition);
        }
        lastDpadRight = gamepad1.dpad_right;

        // D-Pad Left: Decrease RIGHT trapdoor position
        if (gamepad1.dpad_left && !lastDpadLeft) {
            rightPosition = Math.max(0.0, rightPosition - stepSize);
            rightTrapdoor.setPosition(rightPosition);
        }
        lastDpadLeft = gamepad1.dpad_left;

        // Left Trigger: Decrease step size
        if (gamepad1.left_trigger > 0.5) {
            stepSize = Math.max(0.001, stepSize / 2);
        }

        // Right Trigger: Increase step size
        if (gamepad1.right_trigger > 0.5) {
            stepSize = Math.min(0.1, stepSize * 2);
        }

        // === Telemetry ===
        telemetry.addData("=== TRAPDOOR TEST ===", "");
        telemetry.addData("", "");
        telemetry.addData("LEFT Trapdoor", leftOpen ? "OPEN" : "CLOSED");
        telemetry.addData("  Position", "%.3f (Open=%.2f, Closed=%.2f)", leftPosition, LEFT_OPEN, LEFT_CLOSED);
        telemetry.addData("", "");
        telemetry.addData("RIGHT Trapdoor", rightOpen ? "OPEN" : "CLOSED");
        telemetry.addData("  Position", "%.3f (Open=%.2f, Closed=%.2f)", rightPosition, RIGHT_OPEN, RIGHT_CLOSED);
        telemetry.addData("", "");
        telemetry.addData("Step Size", "%.3f", stepSize);
        telemetry.addData("", "");
        telemetry.addData("=== CONTROLS ===", "");
        telemetry.addData("X", "OPEN LEFT");
        telemetry.addData("Y", "CLOSE LEFT");
        telemetry.addData("B", "OPEN RIGHT");
        telemetry.addData("A", "CLOSE RIGHT");
        telemetry.addData("Left Bumper", "OPEN BOTH");
        telemetry.addData("Right Bumper", "CLOSE BOTH");
        telemetry.addData("DPad Up/Down", "Fine-tune LEFT");
        telemetry.addData("DPad Left/Right", "Fine-tune RIGHT");
        telemetry.addData("Triggers", "Change step size");
        telemetry.update();
    }
}
