package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

@TeleOp(name = "nsbhsv4_tc")
public class nsbhsv4_tc extends LinearOpMode {
  private Servo leftServo;
  private Servo rightServo;
  private DcMotor leftMotor; // Front left
  private DcMotor rightMotor; // Front right
  private DcMotorEx flywheel;
  private DcMotor intake;
  double driveSensitivity = 1.0;
  double driveSpeedScale = 1.0;
  boolean lastOptionState = false;
  boolean intake_on = false;
  int indexerPos = 0;
  double leftPower;
  double rightPower;
  boolean flywheel_on = false;
  boolean lastYState = false;

  // intake state: 0 = off, 1 = forward (RT), -1 = reverse (LT)
  int intakeState = 0;
  boolean lastRT = false;
  boolean lastLT = false;

  static final double FLYWHEEL_TARGET_VELOCITY = 1300;
  static final double TRIGGER_THRESHOLD = 0.5;

  @Override
  public void runOpMode() {
    ElapsedTime runtime = new ElapsedTime();

    leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
    rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");
    leftServo = hardwareMap.get(Servo.class, "leftServo");
    rightServo = hardwareMap.get(Servo.class, "rightServo");
    flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
    intake = hardwareMap.get(DcMotor.class, "intake");
    leftMotor.setDirection(DcMotor.Direction.REVERSE);
    rightMotor.setDirection(DcMotor.Direction.FORWARD);
    intake.setDirection(DcMotor.Direction.REVERSE);

    flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    telemetry.addData("Status", "Initialized - 2-Wheel Tank Drive");
    telemetry.addData("Drive Control", "gp1 sticks only");
    telemetry.addData("RB", "Flywheel toggle  | LB: index  (gp1 or gp2)");
    telemetry.addData("RT", "Intake fwd toggle  | LT: intake rev toggle  (gp1 or gp2)");
    telemetry.update();

    waitForStart();
    runtime.reset();

    while (opModeIsActive()) {
      // drive: gamepad1 only
      leftPower = -gamepad1.left_stick_y * driveSensitivity;
      rightPower = -gamepad1.right_stick_y * driveSensitivity;

      // indexing: both gamepads, leftup
      boolean indexBtn = gamepad1.left_bumper || gamepad2.left_bumper;
      indexerPos = Boolean.compare(indexBtn, false);

      // chassis speed toggle: gamepad1 only (options)
      boolean currentOptionState = gamepad1.options;
      if (currentOptionState && !lastOptionState) {
        driveSpeedScale = (driveSpeedScale == 1.0) ? 0.5 : 1.0;
      }
      lastOptionState = currentOptionState;

      leftMotor.setPower(leftPower * driveSpeedScale);
      rightMotor.setPower(rightPower * driveSpeedScale);
      leftServo.setPosition(1 - indexerPos);
      rightServo.setPosition(indexerPos);

      // flywheel toggle, both gamepad
      boolean flywheelBtn = gamepad1.right_bumper || gamepad2.right_bumper;
      if (flywheelBtn && !lastYState) {
        flywheel_on = !flywheel_on;
      }
      lastYState = flywheelBtn;

      if (flywheel_on) {
        flywheel.setVelocity(FLYWHEEL_TARGET_VELOCITY);
      } else {
        flywheel.setVelocity(0);
      }

      // intake toggles: RightDown (RT) = forward, LeftDown (LT) = reverse, both gamepads work
      // press same trigger again to turn off and press opposite to switch direction
      boolean rt = (gamepad1.right_trigger > TRIGGER_THRESHOLD) || (gamepad2.right_trigger > TRIGGER_THRESHOLD);
      boolean lt = (gamepad1.left_trigger > TRIGGER_THRESHOLD) || (gamepad2.left_trigger > TRIGGER_THRESHOLD);

      if (rt && !lastRT) {
        intakeState = (intakeState == 1) ? 0 : 1;
      }
      lastRT = rt;

      if (lt && !lastLT) {
        intakeState = (intakeState == -1) ? 0 : -1;
      }
      lastLT = lt;

      intake.setPower(intakeState);
      intake_on = (intakeState != 0);

      telemetry.addData("Status", "Runtime: " + runtime);
      telemetry.addData("Chassis Speed", (int)(driveSpeedScale * 100) + "% (gp1 Option)");
      telemetry.addData("Final Power", "Left: " + JavaUtil.formatNumber(leftPower * driveSpeedScale, 4, 2)
              + ", Right: " + JavaUtil.formatNumber(rightPower * driveSpeedScale, 4, 2));
      telemetry.addData("Flywheel On", flywheel_on);
      telemetry.addData("Flywheel Actual", flywheel.getVelocity());
      telemetry.addData("Intake State", intakeState == 1 ? "FWD" : intakeState == -1 ? "REV" : "OFF");
      telemetry.update();
    }
  }
}
