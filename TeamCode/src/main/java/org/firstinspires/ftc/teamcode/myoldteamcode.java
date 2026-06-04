package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.DriveHelper;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import java.util.List;

/**
 * DriveHelper Function Test Program
 * Tests all main method functionalities
 */
@Autonomous(name="autoRedFront")
public class autoRedFront extends LinearOpMode {

    private DriveHelper dh;
    private ElapsedTime runtime = new ElapsedTime();
    
    // Ball color definition and recording
    private enum BallColor { GREEN, PURPLE, UNKNOWN }
    private BallColor lastDetectedLeftColor = BallColor.UNKNOWN;
    private BallColor lastDetectedRightColor = BallColor.UNKNOWN;
    
    // Color recognition parameters
    private static final float PURPLE_H_MIN = 200f, PURPLE_H_MAX = 300f;
    private static final float GREEN_H_MIN  =  80f, GREEN_H_MAX  = 170f;
    private static final double IN_THRESH_CM = 4.0; // Distance threshold (cm)
    
    // Motor definitions
    private DcMotorEx fly;     // Flywheel shooting motor
    private DcMotor left;    // Left launcher control motor
    private DcMotor right;   // Right launcher control motor
    private DcMotor intake;
    NormalizedColorSensor cl;
    NormalizedColorSensor cr;
    
    private static double PowerSpeed = 0.50; // important 1
    
    
    // AprilTag vision system
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTagProcessor;

    /**
     * Repeatedly execute DriveHelper method until success
     * @param method Method to repeatedly execute (using lambda expression)
     */
    private void block(java.util.function.Supplier<Boolean> method) {
        while (opModeIsActive() && !method.get()) {
            sleep(20);
        }
    }

    /**
     * Classify ball color by Hue
     */
    private BallColor classifyColorByHue(float hue) {
        if (inHueRange(hue, PURPLE_H_MIN, PURPLE_H_MAX)) return BallColor.PURPLE;
        if (inHueRange(hue, GREEN_H_MIN,  GREEN_H_MAX )) return BallColor.GREEN;
        return BallColor.UNKNOWN;
    }

    private boolean inHueRange(float h, float min, float max) {
        return h >= min && h <= max;
    }

    /**
     * Turn on light and perform one color sample, returns [color, Hue]
     */
    private BallColor sampleOnceWithLight(NormalizedColorSensor sensor, float[] outHsv) {
        boolean lightWasOn = false;
        if (sensor instanceof SwitchableLight) {
            SwitchableLight light = (SwitchableLight) sensor;
            // Record previous state (some SDK hardware doesn't support reading, ignore exceptions)
            try { lightWasOn = true; } catch (Exception ignored) {}
            try { light.enableLight(true); } catch (Exception ignored) {}
        }

        NormalizedRGBA rgba = sensor.getNormalizedColors();
        Color.colorToHSV(rgba.toColor(), outHsv);
        float hue = outHsv[0];
        BallColor color = classifyColorByHue(hue);

        if (sensor instanceof SwitchableLight) {
            try { ((SwitchableLight) sensor).enableLight(false); } catch (Exception ignored) {}
        }
        return color;
    }

    @Override
    public void runOpMode() {
        // ============================================================
        // 0. Initialize DriveHelper
        // ============================================================
        telemetry.addData("Status", "Initializing DriveHelper...");
        telemetry.update();
        
        dh = DriveHelper.initialize(
            this,
            "bl",   // Left drive motor name
            "br",   // Right drive motor name
            "imu"   // IMU name
        );
        
        // Initialize other motors
        fly = hardwareMap.get(DcMotorEx.class, "fly");     // Flywheel shooting motor
        left = hardwareMap.get(DcMotor.class, "left");   // Left launcher control motor
        right = hardwareMap.get(DcMotor.class, "right"); // Right launcher control motor
        intake = hardwareMap.get(DcMotor.class, "intake");
        
        // Set motor directions (refer to v2_tc.java settings)
        fly.setDirection(DcMotor.Direction.FORWARD);   // Flywheel motor forward
        left.setDirection(DcMotor.Direction.FORWARD);  // Left motor forward
        right.setDirection(DcMotor.Direction.REVERSE); // Right motor forward

        cl = hardwareMap.get(NormalizedColorSensor.class, "cl");
        cr = hardwareMap.get(NormalizedColorSensor.class, "cr");
        
        // Initialize AprilTag vision system
        initAprilTag();
        
        telemetry.addData("Status", "Initialization complete!");
        telemetry.addData("Hint", "Press START to begin");
        telemetry.update();

        // Display IMU status while waiting for start
        while (!isStarted() && !isStopRequested()) {
            dh.displayInitStatus();
            sleep(50);
        }

        waitForStart();
        runtime.reset();

        //Movement calibration
        //If there's error between movement distance and measured distance, use these 2 lines to calibrate
        //Move four grid distances, specifically for adjusting parameter value correction
        //Modify parameter location DriveHelper.java  public static final double WHEEL_DIAMETER_MM
        // dh.prepareForStart(0, 0, 0);
        // block(() -> dh.driveStraight(0.5,2394));

        // ============================================================
        // 1. Autonomous Code
        // ============================================================

        // Initialize motor states
        intake.setPower(0);
        fly.setPower(PowerSpeed);

        // Initial position setup - can be obtained from Onshape or measured manually
        //Note positive/negative coordinates and orientation
        final double startX = 1000, startY = 972, startHeading = -90;  //Red 1 position
        //final double startX = 590, startY = -1500, startHeading = 0; //Red 2 position
        //final double startX = -1350, startY = 972, startHeading = 90; //Blue 1 position
        //final double startX = -590, startY = -1500, startHeading = 0; //Blue 2 position

        // April tag position setting (startX > 0 Red team < 0 Blue team)
        final int tagX = startX > 0 ? 1413 : -1413;
        final int tagY = 1482;

        // Shooting position - modify x y coordinates here (startX > 0 Red team < 0 Blue team)
        final int shootX = startX > 0 ? 590 : -590;
        final int shootY = 972;

        dh.prepareForStart(startX, startY, startHeading); 
        //1 Move to shootXY, can adjust last parameter to increase speed
        intake.setPower(0);
        block(() -> dh.driveToTarget(shootX,shootY,true,1));
        

        //2 Turn to obelisk, can adjust last parameter to increase speed
        block(() -> dh.turnToTarget(0,1820,true,0.4));

        //3 Detect tag 
        int tagId=detectAndPrintTagId();
        telemetry.addData("Status", "Detection complete ID="+ tagId); 
        telemetry.update();

        //4 Turn to basket
        block(() -> dh.turnToTarget(tagX,tagY,true,0.6));
        block(() -> dh.turnByDegrees(0.6,-12)); // important 2 , tuning this
        //block(() -> dh.driveStraight(0.6, 50)); // important 2 , tuning this
        

        //5 Shoot 3 balls 21GPP 22PGP 23PPG
        //Manually preset balls must be fixed, e.g. right 2 purple left 1 green
        //If id 21 then shoot3Balls("left","right","both", 0.55);
        if(tagId==21){
            telemetry.addData("Shooting order", "left-right-both");
            telemetry.update();
            shoot3Balls("left","right","both", PowerSpeed);
        }else if (tagId==22) {
            telemetry.addData("Shooting order", "right-left-both");
            telemetry.update();
            shoot3Balls("right","left","both", PowerSpeed);
        }else{
            telemetry.addData("Shooting order", "right-right-both");
            telemetry.update();
            shoot3Balls("right","right","both", PowerSpeed);
        }
        
        //6
        intake.setPower(1);
        
        block(() -> dh.driveStraight(1,-400));
        block(() -> dh.turnByDegrees(0.6,80));
        block(() -> dh.driveStraight(1,170));
        
        
        
        left.setPower(0.15);
        right.setPower(0.21);
        block(() -> dh.driveStraight(0.2,400));
        
    
        
        block(() -> dh.turnByDegrees(0.6,-100));
        block(() -> dh.driveStraight(1,350));
        
        //7 copy from 5 Shoot 3 balls 21GPP 22PGP 23PPG, and changed
        
        
        telemetry.addData("Shooting order", "both-both-both");
        telemetry.update();
        shoot3Balls("right","both","both", PowerSpeed);
        
        
        
        


        //Below is continued scoring template - untested, needs tuning

        // //6 Collect 3 more balls
        // sleep(1000);
        // block(() -> dh.turnToHeading(0.3, 180));
        // block(() -> dh.driveToTarget(dh.getCurrentX(),300,true,0.5));
        // block(() -> dh.turnToHeading(0.3, 90));
        // get3Ball();

        // //7 Return to front for shooting
        // block(() -> dh.turnToTarget(shootX,shootY,true,0.5));
        // block(() -> dh.driveToTarget(shootX,shootY,true,0.5));
        
        //sleep(3000);
    }

    // Collect 3 balls
    //intake lr rotate, very slow forward for specified distance, stop when distance reached
    //Left and right chambers detect separately, stop rotation when ball is loaded, wait for shooting
    //Ball color (green, purple) stored in lastDetectedLeftColor,lastDetectedRightColor, used later to determine left/right shooting order
    public void get3Ball(){
        intake.setPower(1);
        left.setPower(0.5);
        right.setPower(0.5);
        dh.driveStraight(0.05,500);

        boolean leftSampled = false;
        boolean rightSampled = false;
        float lastLeftHue = Float.NaN;
        float lastRightHue = Float.NaN;
        float[] hsvBuf = new float[3];

        while (opModeIsActive() && !dh.isDriveComplete()) {
            double leftDist = ((DistanceSensor)cl).getDistance(DistanceUnit.CM);
            double rightDist = ((DistanceSensor)cr).getDistance(DistanceUnit.CM);

            if (leftDist <= IN_THRESH_CM) {
                left.setPower(0);
                if (!leftSampled) {
                    BallColor color = sampleOnceWithLight(cl, hsvBuf);
                    lastDetectedLeftColor = color;
                    lastLeftHue = hsvBuf[0];
                    leftSampled = true;
                }
            }

            if (rightDist <= IN_THRESH_CM) {
                right.setPower(0);
                if (!rightSampled) {
                    BallColor color = sampleOnceWithLight(cr, hsvBuf);
                    lastDetectedRightColor = color;
                    lastRightHue = hsvBuf[0];
                    rightSampled = true;
                }
            }

            telemetry.addData("Left distance (cm)", "%.2f", leftDist);
            telemetry.addData("Left color", String.valueOf(lastDetectedLeftColor));
            telemetry.addData("Left Hue", Float.isNaN(lastLeftHue) ? "-" : String.format("%.1f", lastLeftHue));
            telemetry.addData("Right distance (cm)", "%.2f", rightDist);
            telemetry.addData("Right color", String.valueOf(lastDetectedRightColor));
            telemetry.addData("Right Hue", Float.isNaN(lastRightHue) ? "-" : String.format("%.1f", lastRightHue));
            telemetry.update();

            sleep(100);
        }
        left.setPower(0);
        right.setPower(0);
    }

    /**
     * Shoot 3 balls function (custom order)
     * @param ball1 1st ball shooting method: "left", "right", "both"
     * @param ball2 2nd ball shooting method: "left", "right", "both"
     * @param ball3 3rd ball shooting method: "left", "right", "both"
     * @param targetFlySpeed Target flywheel speed (0.0-1.0)
     */
    public void shoot3Balls(String ball1, String ball2, String ball3, double targetFlySpeed) {
        telemetry.addData("Status", "Starting to shoot 3 balls");
        telemetry.addData("Shooting order", "1st:%s 2nd:%s 3rd:%s", ball1, ball2, ball3);
        telemetry.addData("Flywheel speed", "%.2f", targetFlySpeed);
        telemetry.update();
        
        // Set intake speed to 100%
        intake.setPower(1.0);
        
        // Set fly speed
        fly.setPower(targetFlySpeed);
        
        // Wait for fly speed to reach target speed
        waitForFlySpeed(targetFlySpeed);
        
        // Shoot 1st ball
        shootBall(1, ball1, targetFlySpeed);
        //sleep(3000);
        
        // Wait for fly speed to recover to target speed
        waitForFlySpeed(targetFlySpeed);
        
        // Shoot 2nd ball
        shootBall(2, ball2, targetFlySpeed);
        //sleep(3000);
        

        // Wait for fly speed to recover to target speed
        waitForFlySpeed(targetFlySpeed);
        
        // Shoot 3rd ball
        shootBall(3, ball3, targetFlySpeed);
        
        // Stop all motors
        intake.setPower(0);
        fly.setPower(0);
        left.setPower(0);
        right.setPower(0);
        
        telemetry.addData("Status", "Shooting 3 balls complete");
        telemetry.update();
    }

    /**
     * Shoot a single ball
     * @param ballNumber Ball number (for display)
     * @param side Shooting method: "left", "right", "both"
     * @param targetFlySpeed Target flywheel speed
     */
    private void shootBall(int ballNumber, String side, double targetFlySpeed) {
        String sideName = getSideName(side);
        telemetry.addData("Status", "Shooting ball %d (%s)", ballNumber, sideName);
        telemetry.update();
        
        // Set motor power according to shooting method
        if ("left".equalsIgnoreCase(side)) {
            left.setPower(0.5);
            right.setPower(0);
        } else if ("right".equalsIgnoreCase(side)) {
            left.setPower(0);
            right.setPower(0.5);
        } else if ("both".equalsIgnoreCase(side)) {
            left.setPower(0.5);
            right.setPower(0.5);
        } else {
            telemetry.addData("Error", "Unknown shooting method: %s", side);
            telemetry.update();
            return;
        }
        
        // Wait for ball to be shot
        waitForBallShot(targetFlySpeed);
        
        // Stop shooting motors
        left.setPower(0);
        right.setPower(0);

        // Clear corresponding side color after shooting, prepare for next collection
        if ("left".equalsIgnoreCase(side)) {
            lastDetectedLeftColor = BallColor.UNKNOWN;
        } else if ("right".equalsIgnoreCase(side)) {
            lastDetectedRightColor = BallColor.UNKNOWN;
        } else if ("both".equalsIgnoreCase(side)) {
            lastDetectedLeftColor = BallColor.UNKNOWN;
            lastDetectedRightColor = BallColor.UNKNOWN;
        }
    }

    /**
     * Get display name for shooting method
     */
    private String getSideName(String side) {
        if ("left".equalsIgnoreCase(side)) {
            return "Left";
        } else if ("right".equalsIgnoreCase(side)) {
            return "Right";
        } else if ("both".equalsIgnoreCase(side)) {
            return "Both";
        } else {
            return "Unknown";
        }
    }

    /**
     * Wait for flywheel speed to reach target speed
     */
    private void waitForFlySpeed(double targetSpeed) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        
        // HD Hex Motor max speed: 6000 RPM
        // Encoder resolution: 28 CPR (Counts Per Revolution)
        // Max speed = 6000 RPM / 60 * 28 = 2,800 ticks/second
        double maxVelocity = 2800.0; // ticks per second
        double targetVelocity = targetSpeed * maxVelocity;
        
        while (opModeIsActive() && timer.seconds() < 3.0) {
            double currentSpeed = Math.abs(fly.getVelocity());
            
            // Consider reached when speed is 85% or more of target speed
            if (currentSpeed >= targetVelocity * 0.85) {
                telemetry.addData("Flywheel speed", "Target speed reached");
                telemetry.update();
                return;
            }
            
            telemetry.addData("Flywheel speed", "%.0f / %.0f ticks/s", currentSpeed, targetVelocity);
            telemetry.update();
            sleep(20);
        }
        
        telemetry.addData("Warning", "Flywheel acceleration timeout");
        telemetry.update();
    }

    /**
     * Wait for ball to be shot (by detecting sudden drop in fly speed)
     * @param targetSpeed Target speed
     */
    private void waitForBallShot(double targetSpeed) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();
        
        double previousSpeed = Math.abs(fly.getVelocity());
        
        while (opModeIsActive() && timer.seconds() < 4.0) {
            double currentSpeed = Math.abs(fly.getVelocity());
            
            // Detect sudden speed drop (drop more than 10%)
            if (currentSpeed < previousSpeed * 0.95) {
                telemetry.addData("Shooting status", "Ball shot");
                telemetry.update();
                return; // Ball has been shot
            }
            
            previousSpeed = currentSpeed;
            sleep(20);
        }
        
        telemetry.addData("Shooting status", "Timeout");
        telemetry.update();
    }

    /**
     * Initialize AprilTag vision system
     */
    private void initAprilTag() {
        telemetry.addData("Status", "Initializing AprilTag...");
        telemetry.update();
        
        // Create AprilTag processor
        aprilTagProcessor = new AprilTagProcessor.Builder()
            .setDrawTagID(true)
            .setDrawTagOutline(true)
            .setDrawAxes(true)
            .setDrawCubeProjection(true)
            .build();
        
        // Create VisionPortal (using USB camera only)
        try {
            WebcamName webcam = hardwareMap.get(WebcamName.class, "Webcam 1");
            visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTagProcessor)
                .setCameraResolution(new android.util.Size(640, 480))
                .enableLiveView(true)
                .setAutoStopLiveView(false)
                .build();
            telemetry.addData("AprilTag", "External camera initialization successful");
        } catch (Exception e) {
            telemetry.addData("AprilTag", "Initialization failed: " + e.getMessage());
            aprilTagProcessor = null;
        }
        telemetry.update();
        
        // Wait for camera to start
        if (visionPortal != null) {
            telemetry.addData("Status", "Waiting for camera to start...");
            telemetry.update();
            //sleep(2000);
        }
    }

    /**
     * Detect AprilTag ID, print and return immediately upon detection
     * Detection range: 21/22/23 and 100-105
     */
    public int detectAndPrintTagId() {
        if (aprilTagProcessor == null) {
            telemetry.addData("Error", "AprilTag processor not initialized");
            telemetry.update();
            return 0;
        }

        // Wait for camera to start
        if (visionPortal != null && visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Status", "Waiting for camera to start...");
            telemetry.update();
            ElapsedTime waitTimer = new ElapsedTime();
            waitTimer.reset();
            
            while (opModeIsActive() && 
                   visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING && 
                   waitTimer.seconds() < 5.0) {
                sleep(100);
            }
        }

        ElapsedTime detectionTimer = new ElapsedTime();
        detectionTimer.reset();
        
        // Detect for up to 30 seconds
        while (opModeIsActive() && detectionTimer.seconds() < 30.0) {
            List<AprilTagDetection> detections = aprilTagProcessor.getDetections();
            
            if (detections != null && !detections.isEmpty()) {
                for (AprilTagDetection detection : detections) {
                    int tagId = detection.id;
                    
                    // Detect tags 21/22/23
                    if (tagId == 21 || tagId == 22 || tagId == 23) {
                        telemetry.addData("=== Tag 21-23 Detected ===", "");
                        telemetry.addData("Tag ID", "%d", tagId);
                        telemetry.addData("Center position", "X: %.1f, Y: %.1f", detection.center.x, detection.center.y);
                        
                        if (detection.ftcPose != null) {
                            telemetry.addData("Distance", "%.1f inches", detection.ftcPose.range);
                            telemetry.addData("Y offset", "%.1f inches", detection.ftcPose.y);
                            telemetry.addData("X offset", "%.1f inches", detection.ftcPose.x);
                            telemetry.addData("Yaw", "%.1f degrees", Math.toDegrees(detection.ftcPose.yaw));
                        }
                        
                        telemetry.addData("Confidence", "%.2f", detection.decisionMargin);
                        telemetry.update();
                        return tagId; // Return immediately upon detection
                    }
                }
            }
            
            // Display detection progress
            telemetry.addData("Detecting", "Searching for tags...");
            telemetry.addData("Detection time", "%.1f / 30.0 seconds", detectionTimer.seconds());
            telemetry.update();
            sleep(100);
        }
        
        telemetry.addData("Result", "Target tag not detected");
        telemetry.update();
        return 0;
    }
}
