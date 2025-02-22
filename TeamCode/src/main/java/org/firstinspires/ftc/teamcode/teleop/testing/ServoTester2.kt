package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx

@TeleOp(name = "ServoPositionTester2", group = "Basic Chassis")
class ServoTester2 : LinearOpMode() {
    override fun runOpMode() {

        val launcherServo = hardwareMap.servo["launcherServo"]

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        waitForStart()

        telemetry.addData("Status", "Running")

        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        while (opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            if (currentGamepad1.a && !previousGamepad1.a) {
                launcherServo.position += 0.01
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                launcherServo.position -= 0.01
            }

            telemetry.addData("launcher servo position:", launcherServo.position)
            telemetry.update()
        }
    }
}