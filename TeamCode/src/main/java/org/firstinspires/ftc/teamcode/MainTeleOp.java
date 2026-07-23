package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.BaseOpMode;


@TeleOp
public class MainTeleOp extends BaseOpMode {
    private static final boolean IS_ROBOT_CENTRIC = false;

    @Override
    public void init() {
        super.init();
    }

    @Override
    public void loop() {
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        robot.drivetrain.drive(forward, strafe, turn, IS_ROBOT_CENTRIC);

        if (gamepad1.optionsWasPressed()) robot.drivetrain.toggleSlowMode();

        super.loop();
    }
}
