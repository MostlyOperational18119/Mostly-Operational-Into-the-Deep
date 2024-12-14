package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad

@TeleOp(name = "ServoPositionTester", group = "Basic Chassis")
class ServoTester : LinearOpMode() {
    override fun runOpMode() {

        val servo1 = hardwareMap.servo["launcherServo"]
        servo1.position = 0.05

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
                servo1.position += 0.02
                //servo1.power = 1.0
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                servo1.position -= 0.02
                //servo1.power = -1.0
            }
            //else if (currentGamepad1.x && !previousGamepad1.x){
                //servo1.power = 0.0
            //}


            telemetry.addData("Servo1 position:", servo1.position)
            telemetry.update()
        }
    }
}