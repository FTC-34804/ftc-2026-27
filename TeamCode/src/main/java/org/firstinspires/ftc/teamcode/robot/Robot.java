package org.firstinspires.ftc.teamcode.robot;

import com.bylazar.telemetry.JoinedTelemetry;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Prism;

public class Robot {
    public final HardwareMap hardwareMap;
    public final Telemetry telemetry;

    public final Drivetrain drivetrain;
    public final Prism prism;

    public Robot(OpMode opMode) {
        hardwareMap = opMode.hardwareMap;
        telemetry = new JoinedTelemetry(
                PanelsTelemetry.INSTANCE.getFtcTelemetry(),
                opMode.telemetry
        );

        drivetrain = new Drivetrain(this);
        prism = new Prism(this);
    }
}
