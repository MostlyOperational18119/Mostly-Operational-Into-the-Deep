package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "ServoPositionTester", group = "Basic Chassis")
class ServoTester : LinearOpMode() {
    override fun runOpMode() {
        val airplaneServo = hardwareMap.get(Servo::class.java, "rotateServo")

        telemetry.addData("Status", "Initialized")
        telemetry.update()
        waitForStart()
        telemetry.addData("Status", "Running")
        while (opModeIsActive()) {
            if (gamepad1.a) {
                airplaneServo.position += 0.05;
                sleep(100)
            } else if (gamepad1.b) {
                airplaneServo.position -= 0.05;
                sleep(100)
            }
            val servoPosition = airplaneServo.position;
            telemetry.addLine("servo position: $servoPosition")
            telemetry.update()

        }
    }
}