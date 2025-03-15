package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ColorSensor
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor


@TeleOp
class ColorSensorTest : LinearOpMode() {
    // Define a variable for our color sensor
    var color_sensor: ColorSensor? = null

    override fun runOpMode() {
        // Get the color sensor from hardwareMap
        color_sensor = hardwareMap.get(ColorSensor::class.java, "color")
        // Wait for the Play button to be pressed
        waitForStart()
        // While the OpMode is running, update the telemetry values.
        while (opModeIsActive()) {
            telemetry.addData("Light Detected", (color_sensor as OpticalDistanceSensor).lightDetected)
            telemetry.addData("Red", (color_sensor as ColorSensor).red())
            telemetry.addData("Green", (color_sensor as ColorSensor).green())
            telemetry.addData("Blue", (color_sensor as ColorSensor).blue())
            telemetry.update()
        }
    }
}
