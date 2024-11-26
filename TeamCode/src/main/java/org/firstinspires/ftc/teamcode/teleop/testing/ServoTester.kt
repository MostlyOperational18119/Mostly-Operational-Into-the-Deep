package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "ServoPositionTester", group = "Basic Chassis")
class ServoTester : LinearOpMode() {
    override fun runOpMode() {
        val servo = hardwareMap.servo["launcherServo"]

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()
        telemetry.addData("Status", "Running")

        val currentGamepad = Gamepad()
        val previousGamepad = Gamepad()

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad)
            currentGamepad.copy(gamepad1)

            if (currentGamepad.a && !previousGamepad.a) {
                servo.position += 0.01
            } else if (currentGamepad.b && !previousGamepad.b) {
                servo.position -= 0.01
            }

            telemetry.addLine("Servo position: ${servo.position}")
            telemetry.update()

            sleep(50)
        }
    }
}