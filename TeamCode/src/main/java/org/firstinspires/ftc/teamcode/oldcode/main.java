package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class main extends LinearOpMode {

    private DcMotor leftMotor;

    private DcMotor rightMotor;
    private Servo servo;

    double driveSensitivity = 1.0;
    double driveSpeedScale = 1.0;
    boolean lastOptionState = false;

    double leftPower;
    double rightPower;

    @Override
    public void runOpMode() {
        leftMotor = hardwareMap.get(DcMotor.class, "left");
        rightMotor = hardwareMap.get(DcMotor.class, "right");

        leftMotor.setDirection(DcMotor.Direction.FORWARD);
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            leftPower  = -gamepad1.left_stick_y  * driveSensitivity;
            rightPower = -gamepad1.right_stick_y * driveSensitivity;


            boolean currentOptionState = gamepad1.options;
            if (currentOptionState && !lastOptionState) {
                driveSpeedScale = (driveSpeedScale == 1.0) ? 0.5 : 1.0;
            }
            lastOptionState = currentOptionState;

            leftMotor.setPower(leftPower  * driveSpeedScale);
            rightMotor.setPower(rightPower * driveSpeedScale);


            telemetry.addData("Speed", (int)(driveSpeedScale * 100) + "%");
            telemetry.update();
        }
    }
}
