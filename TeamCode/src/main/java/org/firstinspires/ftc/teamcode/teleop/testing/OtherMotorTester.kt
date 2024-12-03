package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

@Disabled
@TeleOp(name = "OtherMotorTester")
class OtherMotorTester : LinearOpMode() {
    override fun runOpMode() {
        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]

        // Motor run variables
        var motorFLRun = false
        var motorFRRun = false
        var motorBLRun = false
        var motorBRRun = false

        val runPower = 0.5

        // Gamepads
        var currentGamepad = Gamepad()
        var previousGamepad = Gamepad()


        waitForStart()

        while (opModeIsActive()) {
            previousGamepad.copy(currentGamepad)
            currentGamepad.copy(gamepad1)

            // Results:
            // A: BL (config FL) done
            // B: FR (config FR)
            // X: FL (config BL)
            // Y: BR (config BR)
            if (currentGamepad.a && !previousGamepad.a) { // motorFL
                motorFLRun = !motorFLRun
                motorFL.power = if (motorFLRun) runPower else 0.0
            }

            if (currentGamepad.b && !previousGamepad.b) { // motorFR
                motorFRRun = !motorFRRun
                motorFR.power = if (motorFRRun) runPower else 0.0
            }

            if (currentGamepad.x && !previousGamepad.x) { // motorBL
                motorBLRun = !motorBLRun
                motorBL.power = if (motorBLRun) runPower else 0.0
            }

            if (currentGamepad.y && !previousGamepad.y) { // motorBR
                motorBRRun = !motorBRRun
                motorBR.power = if (motorBRRun) runPower else 0.0
            }

            telemetry.addLine("motorFL running: $motorFLRun")
            telemetry.addLine("motorFR running: $motorFRRun")
            telemetry.addLine("motorBL running: $motorBLRun")
            telemetry.addLine("motorBR running: $motorBRRun")
            telemetry.addLine("Previous A: ${previousGamepad.a}. Current A: ${currentGamepad.a}")
            println("Previous A: ${previousGamepad.a}. Current A: ${currentGamepad.a}")
            telemetry.update()

            sleep(50)
        }
    }
}