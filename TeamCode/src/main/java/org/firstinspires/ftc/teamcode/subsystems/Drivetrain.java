package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Drivetrain {
    private static final double SLOWMODE_MULTI = 0.5;

    public final Follower follower;
    public final Telemetry telemetry;

    private boolean slowMode = false;

    public Drivetrain(Robot robot) {
        follower = Constants.createFollower(robot.hardwareMap);
        telemetry = robot.telemetry;
    }

    public void drive(double forward, double strafe, double turn, boolean isRobotCentric) {
        if (slowMode) {
            forward *= SLOWMODE_MULTI;
            strafe *= SLOWMODE_MULTI;
            turn *= SLOWMODE_MULTI;
        }

        follower.setTeleOpDrive(forward, strafe, turn, isRobotCentric);
    }

    public void toggleSlowMode() {
        slowMode = !slowMode;
    }

    public Command cycle() {
        return infinite(() -> {
            follower.update();
            Pose current = follower.getPose();

            telemetry.addData("Current X", current.getX());
            telemetry.addData("Current Y", current.getY());
            telemetry.addData("Current Velocity", follower.getVelocity());
            telemetry.addData("Current Heading", current.getHeading());
            telemetry.addData("Slowmode active", slowMode);
        });
    }
}
