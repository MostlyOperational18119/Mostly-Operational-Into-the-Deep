package org.firstinspires.ftc.teamcode.autonomous

import android.graphics.Color
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import org.firstinspires.ftc.teamcode.Methods


@Autonomous(name = "Color", group = "Sensor")
@Disabled
class colorDetect: Methods() {
    override fun runOpMode() {
        var colors: NormalizedRGBA? = null
        var highestValue = 0.0F
        var currentColor = ""
        val ColorSens = hardwareMap.get(NormalizedColorSensor::class.java, "color")
        ColorSens.gain = 50.0F

        waitForStart()
        while (opModeIsActive()) {
            colors = ColorSens.normalizedColors

            if (colors.red <= 0.25 && colors.green <= 0.25 && colors.blue <= 0.25) {
                currentColor = "no block"
            } else {
                highestValue = colors.red
                currentColor = "red"
                if (colors.green >= highestValue) {
                    highestValue = colors.green
                    currentColor = "yellow"
                }
                if (colors.blue > highestValue) {
                    highestValue = colors.blue
                    currentColor = "blue"
                }
            }

            telemetry.addLine(currentColor)
            telemetry.update()
        }
    }
}
