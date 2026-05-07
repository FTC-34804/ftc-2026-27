package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "nsbhsv1_tc")
public class nsbhsv1_tc extends LinearOpMode {

  private DcMotor L;  // Front left
  private DcMotor R;  // Front right

  // ============================================================================================
  // Speed Configuration
  // ============================================================================================
  double driveSensitivity = 1.0;    // Forward/backward sensitivity
  double driveSpeedScale = 1.0;     // 1.0=100%, 0.5=50% (Option button toggles)
  boolean lastOptionState = false;
  // ============================================================================================

  double leftPower;
  double rightPower;

  /**
   * 4-wheel tank drive TeleOp.
   * - Left stick Y: controls left side (front left + back left)
   * - Right stick Y: controls right side (front right + back right)
   * - Options button: toggles chassis speed between 50% and 100%
   */
  @Override
  public void runOpMode() {
    ElapsedTime runtime = new ElapsedTime();

    // Initialize motors
    L = hardwareMap.get(DcMotor.class, "L");
    R = hardwareMap.get(DcMotor.class, "R");

    // ########################################################################################
    // Motor Direction Settings - adjust if any wheel spins the wrong way
    // ########################################################################################
    L.setDirection(DcMotor.Direction.FORWARD);
    R.setDirection(DcMotor.Direction.REVERSE);

    telemetry.addData("Status", "Initialized - 2-Wheel Tank Drive");
    telemetry.addData("Drive Control", "Left stick = left side, Right stick = right side");
    telemetry.addData("Options", "Toggle chassis speed 50% / 100%");
    telemetry.update();

    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {
      // Tank drive: each stick controls one side
      leftPower  = -gamepad1.left_stick_y  * driveSensitivity;
      rightPower = -gamepad1.right_stick_y * driveSensitivity;

      // Options button toggles chassis speed multiplier (50% <-> 100%)
      boolean currentOptionState = gamepad1.options;
      if (currentOptionState && !lastOptionState) {
        driveSpeedScale = (driveSpeedScale == 1.0) ? 0.5 : 1.0;
      }
      lastOptionState = currentOptionState;

      // Send power to all four motors
      L.setPower(leftPower  * driveSpeedScale);
      R.setPower(rightPower * driveSpeedScale);

      // Telemetry
      telemetry.addData("Status", "Runtime: " + runtime);
      telemetry.addData("Movement Sensitivity", (int)(driveSensitivity * 100) + "%");
      telemetry.addData("Chassis Speed", (int)(driveSpeedScale * 100) + "% (Option button toggle)");
      telemetry.addData("Sticks", "Left: " + JavaUtil.formatNumber(gamepad1.left_stick_y, 4, 2)
              + ", Right: " + JavaUtil.formatNumber(gamepad1.right_stick_y, 4, 2));
      telemetry.addData("Final Power", "Left: " + JavaUtil.formatNumber(leftPower * driveSpeedScale, 4, 2)
              + ", Right: " + JavaUtil.formatNumber(rightPower * driveSpeedScale, 4, 2));
      telemetry.update();
    }
  }
}
