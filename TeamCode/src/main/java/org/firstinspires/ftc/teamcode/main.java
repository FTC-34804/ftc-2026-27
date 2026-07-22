package org.firstinspires.ftc.teamcode;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@TeleOp
public class main extends OpMode {

    /* ================================
      Auto / Drive
      (See PedroPathing documentation)
    ================================ */
    private Follower follower;
    private static final Pose startingPose = new Pose();
    private Supplier<PathChain> pathChain;
    // In automated driving mode.
    private boolean automated = false;


    /* ==========================
      Telemetry
      (See Panels documentation)
    ========================== */
    private TelemetryManager telemetryM;


    private static final boolean ROBOT_CENTRIC = true;
    double driveSpeedScale = 1.0;
    boolean lastOptionState = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();

        pathChain = Common::todo; // TODO

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void start() {
        follower.startTeleOpDrive();
    }

    @Override
    public void loop() {
        follower.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        boolean currentOptionState = gamepad1.options;
        if (currentOptionState && !lastOptionState) {
            driveSpeedScale = (driveSpeedScale == 1.0) ? 0.5 : 1.0;
        }
        lastOptionState = currentOptionState;

        if (!automated) {
            follower.setTeleOpDrive(
                    forward * driveSpeedScale,
                    strafe * driveSpeedScale,
                    turn * driveSpeedScale,
                    ROBOT_CENTRIC
            );
        }

        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automated = true;
        }

        telemetryM.addData("Position", follower.getPose());
        telemetryM.addData("velocity", follower.getVelocity());
        telemetryM.addData("automated", automated);
        telemetryM.update(telemetry);
    }
}
