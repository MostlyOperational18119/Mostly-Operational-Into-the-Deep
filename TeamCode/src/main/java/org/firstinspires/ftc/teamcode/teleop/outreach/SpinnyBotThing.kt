package org.firstinspires.ftc.teamcode.teleop.outreach

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

@Suppress("Unused", "SpellCheckingInspection")
@TeleOp(name = "SpinnyBotThing")
@Disabled

class SpinnyBotThing : LinearOpMode() {
    override fun runOpMode() {
        val spinMotor = hardwareMap.dcMotor["spinMotor"]
        val testMotor = hardwareMap.dcMotor["testMotor"]

        // Gamepad
        val gamepad1Current = Gamepad()
        val gamepad1Previous = Gamepad()

        // Other vars
        var spinPower = 0.8
        var doSpin = false
        var doSpinTestMotor = false

        telemetry.addLine("Init done")
        telemetry.update()

        waitForStart()

        while (opModeIsActive()) {
            gamepad1Previous.copy(gamepad1Current)
            gamepad1Current.copy(gamepad1)

            if (gamepad1Current.a && !gamepad1Previous.a) {
                doSpin = true
            }

            if (gamepad1Current.b && !gamepad1Previous.b) {
                doSpin = false
            }

            if (gamepad1Current.y && !gamepad1Previous.y) {
                spinPower = -spinPower
            }

            // Test motor
            if (gamepad1Current.x && !gamepad1Previous.x) {
                doSpinTestMotor = !doSpinTestMotor
                testMotor.power = if (doSpinTestMotor) 1.0 else 0.0
            }

            if (gamepad1Current.dpad_up && !gamepad1Previous.dpad_up) {
                spinPower += 0.02
            }

            if (gamepad1Current.dpad_down && !gamepad1Previous.dpad_down) {
                spinPower -= 0.02
            }

            spinMotor.power = if (doSpin) spinPower else 0.0

            telemetry.addLine("Spin power $spinPower")
            telemetry.addLine("Spin motor power ${spinMotor.power}")
            telemetry.update()

            sleep(30)
        }
    }
}