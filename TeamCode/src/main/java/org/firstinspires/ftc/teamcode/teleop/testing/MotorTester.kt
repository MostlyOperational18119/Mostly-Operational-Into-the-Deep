package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "MotorTester", group = "ZZZZZZZZZ")
class MotorTester : LinearOpMode() {
    override fun runOpMode() {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val motor = hardwareMap.get(DcMotor::class.java, "HorizontalSlide")
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()
        while (opModeIsActive()) {
            var leftX = gamepad1.left_stick_x.toDouble()

            if (leftX >= 0.5) {
                motor.targetPosition += 10
                motor.power = 0.2
            } else if (leftX <= -0.5) {
                motor.targetPosition -= 10
                motor.power = -0.2
            }

            telemetry.addData("BR Power:  ", motor.power)
            telemetry.addData("BR Position ", motor.targetPosition)
            telemetry.addData("Left Stick X:  ", gamepad1.left_stick_x)
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}