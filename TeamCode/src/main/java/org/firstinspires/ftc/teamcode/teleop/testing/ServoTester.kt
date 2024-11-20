package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "ServoPositionTester", group = "Basic Chassis")
class ServoTester : LinearOpMode() {
    override fun runOpMode() {
        val Servo = hardwareMap.get(Servo::class.java, "rotateServo")

        telemetry.addData("Status", "Initialized")
        telemetry.update()
        waitForStart()
        telemetry.addData("Status", "Running")
        while (opModeIsActive()) {
            if (gamepad1.a) {
                Servo.position += 0.05;
                sleep(100)
            } else if (gamepad1.b) {
                Servo.position -= 0.05;
                sleep(100)
            }
            val servoPosition = Servo.position;
            telemetry.addLine("servo position: $servoPosition")
            telemetry.update()

        }
    }
}