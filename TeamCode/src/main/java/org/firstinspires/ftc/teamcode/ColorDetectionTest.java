package org.firstinspires.ftc.teamcode;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.Locale;

/**
 * This OpMode uses a webcam to detect colors on the left and right sides
 * of the bottom 20% of the camera view and determines if the color is
 * green, purple, or neither.
 */
@TeleOp(name = "Color Detection Test", group = "Test")
public class ColorDetectionTest extends LinearOpMode {

    // Custom Vision Processor for color detection
    private ColorRegionProcessor colorProcessor;
    private VisionPortal visionPortal;

    @Override
    public void runOpMode() {
        // Create the custom processor
        colorProcessor = new ColorRegionProcessor();

        // Build the vision portal with the webcam
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .addProcessor(colorProcessor)
                .enableLiveView(true)  // Enable camera preview on Driver Station
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)  // Better for streaming
                .build();

        telemetry.setMsTransmissionInterval(50);
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        telemetry.addLine("Color Detection Test Initialized");
        telemetry.addLine("Analyzing middle 45% of camera view");
        telemetry.addLine("(40% top, 15% bottom, 10% middle gap)");
        telemetry.addLine("Press PLAY to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Get the analysis results
            ColorRegionProcessor.AnalysisResult result = colorProcessor.getAnalysis();

            telemetry.addLine("=== Color Detection Test ===\n");

            // Left side analysis
            telemetry.addLine("--- LEFT SIDE ---");
            telemetry.addData("  Avg RGB", String.format(Locale.US, "(%3d, %3d, %3d)",
                    result.leftRGB[0], result.leftRGB[1], result.leftRGB[2]));
            telemetry.addData("  Avg HSV", String.format(Locale.US, "(%3d, %3d, %3d)",
                    result.leftHSV[0], result.leftHSV[1], result.leftHSV[2]));
            telemetry.addData("  Detected", result.leftColor);

            telemetry.addLine("");

            // Right side analysis
            telemetry.addLine("--- RIGHT SIDE ---");
            telemetry.addData("  Avg RGB", String.format(Locale.US, "(%3d, %3d, %3d)",
                    result.rightRGB[0], result.rightRGB[1], result.rightRGB[2]));
            telemetry.addData("  Avg HSV", String.format(Locale.US, "(%3d, %3d, %3d)",
                    result.rightHSV[0], result.rightHSV[1], result.rightHSV[2]));
            telemetry.addData("  Detected", result.rightColor);

            telemetry.addLine("\n--- SUMMARY ---");
            telemetry.addData("LEFT", result.leftColor);
            telemetry.addData("RIGHT", result.rightColor);

            telemetry.update();
            sleep(50);
        }

        visionPortal.close();
    }

    /**
     * Custom VisionProcessor that analyzes the bottom 20% of the frame
     * and determines the average color on left and right halves.
     */
    public static class ColorRegionProcessor implements VisionProcessor {

        // Analysis result holder
        private volatile AnalysisResult analysisResult = new AnalysisResult();

        // Mats for processing
        private final Mat hsvMat = new Mat();

        // Region rectangles (will be set in init based on frame size)
        private Rect leftRect;
        private Rect rightRect;

        // Color detection thresholds (HSV values)
        // Green: Hue 35-85, Saturation > 50, Value > 50
        // Purple: Hue 120-160, Saturation > 30, Value > 30

        // HSV ranges for green (OpenCV uses H: 0-180, S: 0-255, V: 0-255)
        private static final int GREEN_HUE_MIN = 70;
        private static final int GREEN_HUE_MAX = 100;
        private static final int GREEN_SAT_MIN = 50;
        private static final int GREEN_VAL_MIN = 50;

        // HSV ranges for purple
        private static final int PURPLE_HUE_MIN = 120;
        private static final int PURPLE_HUE_MAX = 160;
        private static final int PURPLE_SAT_MIN = 30;
        private static final int PURPLE_VAL_MIN = 30;

@Override
        public void init(int width, int height, CameraCalibration calibration) {
            // No horizontal borders (full width)
            // 40% top border, 15% bottom border
            // 10% gap in the middle between left and right regions

            int topBorder = (int) (height * 0.40);          // 40% from top
            int bottomBorder = (int) (height * 0.15);       // 15% from bottom
            int middleGap = (int) (width * 0.10);           // 10% gap in middle

            int regionY = topBorder;                        // Start at 40% from top
            int regionHeight = height - topBorder - bottomBorder;  // Middle 45% of screen

            // Each side gets 45% of width (100% - 10% middle gap = 90%, split in half)
            int sideWidth = (width - middleGap) / 2;

            // Left region: from left edge to middle gap
            leftRect = new Rect(0, regionY, sideWidth, regionHeight);

            // Right region: from after middle gap to right edge
            rightRect = new Rect(sideWidth + middleGap, regionY, sideWidth, regionHeight);
        }

        @Override
        public Object processFrame(Mat frame, long captureTimeNanos) {
            // Convert to HSV
            Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);

            // Extract left and right regions from RGB and HSV
            Mat leftRegion = frame.submat(leftRect);
            Mat rightRegion = frame.submat(rightRect);
            Mat leftHsvRegion = hsvMat.submat(leftRect);
            Mat rightHsvRegion = hsvMat.submat(rightRect);

            // Calculate average colors
            Scalar leftAvgRgb = Core.mean(leftRegion);
            Scalar rightAvgRgb = Core.mean(rightRegion);
            Scalar leftAvgHsv = Core.mean(leftHsvRegion);
            Scalar rightAvgHsv = Core.mean(rightHsvRegion);

            // Create new result
            AnalysisResult result = new AnalysisResult();

            // Store RGB values
            result.leftRGB[0] = (int) leftAvgRgb.val[0];
            result.leftRGB[1] = (int) leftAvgRgb.val[1];
            result.leftRGB[2] = (int) leftAvgRgb.val[2];

            result.rightRGB[0] = (int) rightAvgRgb.val[0];
            result.rightRGB[1] = (int) rightAvgRgb.val[1];
            result.rightRGB[2] = (int) rightAvgRgb.val[2];

            // Store HSV values
            result.leftHSV[0] = (int) leftAvgHsv.val[0];
            result.leftHSV[1] = (int) leftAvgHsv.val[1];
            result.leftHSV[2] = (int) leftAvgHsv.val[2];

            result.rightHSV[0] = (int) rightAvgHsv.val[0];
            result.rightHSV[1] = (int) rightAvgHsv.val[1];
            result.rightHSV[2] = (int) rightAvgHsv.val[2];

            // Determine colors
            result.leftColor = determineColor(result.leftHSV);
            result.rightColor = determineColor(result.rightHSV);

            // Draw rectangles on frame for visualization
            // Left region - draw border based on detected color
            Imgproc.rectangle(frame, leftRect, getColorScalar(result.leftColor), 3);
            // Right region - draw border based on detected color
            Imgproc.rectangle(frame, rightRect, getColorScalar(result.rightColor), 3);

            // Store result
            analysisResult = result;

            return null;
        }

        @Override
        public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight,
                                float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
            // Draw labels on the preview
            Paint paint = new Paint();
            paint.setTextSize(30 * scaleCanvasDensity);
            paint.setAntiAlias(true);

            // Left label
            paint.setColor(getAndroidColor(analysisResult.leftColor));
            float leftX = (leftRect.x + leftRect.width / 2.0f) * scaleBmpPxToCanvasPx;
            float leftY = (leftRect.y + leftRect.height / 2.0f) * scaleBmpPxToCanvasPx;
            canvas.drawText(analysisResult.leftColor, leftX - 50, leftY, paint);

            // Right label
            paint.setColor(getAndroidColor(analysisResult.rightColor));
            float rightX = (rightRect.x + rightRect.width / 2.0f) * scaleBmpPxToCanvasPx;
            float rightY = (rightRect.y + rightRect.height / 2.0f) * scaleBmpPxToCanvasPx;
            canvas.drawText(analysisResult.rightColor, rightX - 50, rightY, paint);
        }

        /**
         * Determine if the color is GREEN, PURPLE, or NEITHER based on HSV values.
         */
        private String determineColor(int[] hsv) {
            int hue = hsv[0];
            int sat = hsv[1];
            int val = hsv[2];

            // Check for green
            if (hue >= GREEN_HUE_MIN && hue <= GREEN_HUE_MAX
                    && sat >= GREEN_SAT_MIN && val >= GREEN_VAL_MIN) {
                return "GREEN";
            }

            // Check for purple
            if (hue >= PURPLE_HUE_MIN && hue <= PURPLE_HUE_MAX
                    && sat >= PURPLE_SAT_MIN && val >= PURPLE_VAL_MIN) {
                return "PURPLE";
            }

            return "NEITHER";
        }

        /**
         * Get OpenCV Scalar color for drawing rectangles.
         */
        private Scalar getColorScalar(String color) {
            switch (color) {
                case "GREEN":
                    return new Scalar(0, 255, 0);  // Green
                case "PURPLE":
                    return new Scalar(128, 0, 128);  // Purple
                default:
                    return new Scalar(255, 255, 255);  // White for neither
            }
        }

        /**
         * Get Android Color for drawing text.
         */
        private int getAndroidColor(String color) {
            switch (color) {
                case "GREEN":
                    return Color.GREEN;
                case "PURPLE":
                    return Color.MAGENTA;  // Close to purple
                default:
                    return Color.WHITE;
            }
        }

        /**
         * Get the latest analysis result.
         */
        public AnalysisResult getAnalysis() {
            return analysisResult;
        }

        /**
         * Data class to hold analysis results.
         */
        public static class AnalysisResult {
            public int[] leftRGB = new int[3];
            public int[] rightRGB = new int[3];
            public int[] leftHSV = new int[3];
            public int[] rightHSV = new int[3];
            public String leftColor = "NEITHER";
            public String rightColor = "NEITHER";
        }
    }
}

