package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
class auto extends OpMode {
    final double FULL_SPEED = 1.0;
    final double STOP_SPEED = 0.0;

    private DcMotor leftMotor;
    private DcMotor rightMotor;
    private final ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {
        leftMotor = hardwareMap.get(DcMotor.class, "leftMotor");
        rightMotor = hardwareMap.get(DcMotor.class, "rightMotor");

        leftMotor.setPower(STOP_SPEED);
        rightMotor.setPower(STOP_SPEED);
    }


    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialised");
        telemetry.update();
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        while (runtime.seconds() < 3) {
            leftMotor.setPower(FULL_SPEED);
            rightMotor.setPower(FULL_SPEED);
        }
        leftMotor.setPower(STOP_SPEED);
        rightMotor.setPower(STOP_SPEED);
    }
}