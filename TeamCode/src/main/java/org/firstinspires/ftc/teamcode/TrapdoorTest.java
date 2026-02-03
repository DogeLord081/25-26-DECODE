package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import android.util.Size;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.teamcode.ColorDetectionTest.ColorRegionProcessor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "TrapdoorTest")
public class TrapdoorTest extends OpMode {
    protected Servo leftTrapdoor;
    protected Servo rightTrapdoor;

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        if (gamepad1.x) {
            leftTrapdoor.setPosition(0.0);
            rightTrapdoor.setPosition(0.1);
        }
        if (gamepad1.b) {
            rightTrapdoor.setPosition(0.2);  // Open
            leftTrapdoor.setPosition(0.1);
        }
        if (gamepad1.a) {
            rightTrapdoor.setPosition(0.1);  // Closed
            leftTrapdoor.setPosition(0.1);
        }
        if (gamepad1.y) {
            leftTrapdoor.setPosition(0.0);  // Open
            rightTrapdoor.setPosition(0.2);
        }
    }
}
