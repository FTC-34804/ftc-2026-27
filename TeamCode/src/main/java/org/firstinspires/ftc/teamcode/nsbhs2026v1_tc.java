package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "nsbhs2026v1_tc")
public class nsbhs2026v1_tc extends LinearOpMode {
  private Servo leftServo;
  private Servo rightServo;
  private DcMotor leftFrontMotor; // Front left
  private DcMotor rightFrontMotor; // Front right
  private DcMotor leftBackMotor;
  private DcMotor rightBackMotor;
  double driveSensitivity = 1.0;
  double driveSpeedScale = 1.0;
  boolean lastOptionState = false;
  double leftFrontPower;
  double rightFrontPower;
  double leftBackPower;
  double rightBackPower;

  boolean lastRT = false;
  boolean lastLT = false;

  static final double TRIGGER_THRESHOLD = 0.5;

  @Override
  public void runOpMode() {
    ElapsedTime runtime = new ElapsedTime();

    leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
    rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
    leftBackMotor = hardwareMap.get(DcMotorEx.class, "leftBackMotor");
    rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    leftFrontMotor.setDirection(DcMotor.Direction.REVERSE);
    leftBackMotor.setDirection(DcMotor.Direction.REVERSE);
    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {
      // drive: gamepad1 only
      double y = -gamepad1.left_stick_y; // Forward/Backward
      double x = gamepad1.left_stick_x;  // Left/Right Strafe
      double Magnitude = Math.hypot(x, y);
      if (Magnitude > 1.0) {
        x /= Magnitude;
        y /= Magnitude;
      }
      double rx = gamepad1.right_stick_x; // Rotation
      leftFrontPower = (y+x+rx)*driveSpeedScale;
      rightFrontPower = (y-x-rx)*driveSpeedScale;
      leftBackPower = (y-x+rx)*driveSpeedScale;
      rightBackPower = (y+x-rx)*driveSpeedScale;

      // chassis speed toggle: gamepad1 only (options)
      boolean currentOptionState = gamepad1.options;
      if (currentOptionState && !lastOptionState) {
        driveSpeedScale = (driveSpeedScale == 1.0) ? 0.5 : 1.0;
      }
      lastOptionState = currentOptionState;

      leftFrontMotor.setPower(leftFrontPower);
      leftBackMotor.setPower(leftBackPower);
      rightFrontMotor.setPower(rightFrontPower);
      rightBackMotor.setPower(rightBackPower);
    }
  }
}
