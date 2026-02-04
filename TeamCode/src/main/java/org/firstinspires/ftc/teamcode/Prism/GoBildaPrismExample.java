package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver.LayerHeight;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Prism.Color;
import org.firstinspires.ftc.teamcode.Prism.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.Prism.PrismAnimations;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.concurrent.TimeUnit;

@TeleOp(name="Prism + Rev Color Proximity", group="Linear OpMode")
//@Disabled
public class GoBildaPrismExample extends LinearOpMode {

    // ---- Prism ----
    private GoBildaPrismDriver prism;

    // ---- Sensor ----
    private RevColorSensorV3 colorSense;

    // ---- Animations ----
    private final PrismAnimations.Solid solidGreen = new PrismAnimations.Solid(Color.GREEN);
    private final PrismAnimations.Solid solidBlue  = new PrismAnimations.Solid(Color.BLUE);

    // Tune this threshold for your mounting distance
    // If the sensor reads <= this many mm, we consider "object detected"
    private static final double PROX_THRESHOLD_MM = 75.0;

    @Override
    public void runOpMode() {

        // Hardware map names must match your Robot Config
        prism = hardwareMap.get(GoBildaPrismDriver.class, "prism");
        colorSense = hardwareMap.get(RevColorSensorV3.class, "colorsense");

        // Prism strip length (0..119 = 120 LEDs)
        prism.setStripLength(120);

        // Configure GREEN (full strip)
        solidGreen.setBrightness(100);
        solidGreen.setStartIndex(0);
        solidGreen.setStopIndex(119);

        // Configure BLUE (full strip)
        solidBlue.setBrightness(100);
        solidBlue.setStartIndex(0);
        solidBlue.setStopIndex(119);

        telemetry.addData("Device ID", prism.getDeviceID());
        telemetry.addData("Firmware Version", prism.getFirmwareVersionString());
        telemetry.addData("Hardware Version", prism.getHardwareVersionString());
        telemetry.addData("Power Cycle Count", prism.getPowerCycleCount());
        telemetry.update();

        waitForStart();
        resetRuntime();

        // Track last state so we only update LEDs when it changes
        boolean lastObjectDetected = false;
        boolean firstUpdate = true;

        while (opModeIsActive()) {

            // Proximity reading (mm). Smaller = closer.
            double distMm = colorSense.getDistance(DistanceUnit.MM);
            boolean objectDetected = distMm <= PROX_THRESHOLD_MM;

            // Update Prism only on change (or first loop)
            if (firstUpdate || objectDetected != lastObjectDetected) {
                prism.clearAllAnimations();

                if (objectDetected) {
                    // Object close -> GREEN
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidGreen);
                } else {
                    // Otherwise -> BLUE
                    prism.insertAndUpdateAnimation(LayerHeight.LAYER_0, solidBlue);
                }

                lastObjectDetected = objectDetected;
                firstUpdate = false;
            }

            telemetry.addData("Proximity (mm)", "%.1f", distMm);
            telemetry.addData("Object Detected", objectDetected);
            telemetry.addData("Threshold (mm)", PROX_THRESHOLD_MM);
            telemetry.addLine();
            telemetry.addData("Run Time (Minutes)", prism.getRunTime(TimeUnit.MINUTES));
            telemetry.addData("LED Count", prism.getNumberOfLEDs());
            telemetry.addData("Prism FPS", prism.getCurrentFPS());
            telemetry.update();

            sleep(30);
        }
    }
}
