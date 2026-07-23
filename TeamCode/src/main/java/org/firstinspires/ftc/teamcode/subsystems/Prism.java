package org.firstinspires.ftc.teamcode.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Prism {
    public final GoBildaPrismDriver prismDriver;
    public final Telemetry telemetry;

    public Prism(Robot robot) {
        prismDriver = robot.hardwareMap.get(GoBildaPrismDriver.class, "prism");
        telemetry = robot.telemetry;

        PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.PINK);
        solid.setBrightness(50);
        prismDriver.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_0,
                solid
        );

        PrismAnimations.RainbowSnakes rainbowSnakes = new PrismAnimations.RainbowSnakes();
        rainbowSnakes.setNumberOfSnakes(5);
        rainbowSnakes.setSnakeLength(5);
        rainbowSnakes.setSpacingBetween(5);
        prismDriver.insertAndUpdateAnimation(
                GoBildaPrismDriver.LayerHeight.LAYER_1,
                rainbowSnakes
        );
    }

    public Command cycle() {
        return infinite(() -> {
            telemetry.addData("Prism power cycles", prismDriver.getPowerCycleCount());
            telemetry.addData("Prism FPS", prismDriver.getCurrentFPS());
        }).setEnd(cond -> {
            prismDriver.clearAllAnimations();
        });
    }
}
