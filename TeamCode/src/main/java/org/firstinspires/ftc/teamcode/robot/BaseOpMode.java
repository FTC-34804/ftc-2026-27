package org.firstinspires.ftc.teamcode.robot;

import static com.pedropathing.ivy.Scheduler.schedule;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class BaseOpMode extends OpMode {
    protected Robot robot;
    protected Telemetry telemetry;

    @Override
    public void init() {
        robot = new Robot(this);
        telemetry = robot.telemetry;

        Scheduler.reset();
        schedule(
                robot.drivetrain.cycle(),
                robot.prism.cycle()
        );
    }

    @Override
    public void init_loop() {
        Scheduler.execute();
        telemetry.update();
    }

    @Override
    public void loop() {
        Scheduler.execute();
        telemetry.update();
    }
}
