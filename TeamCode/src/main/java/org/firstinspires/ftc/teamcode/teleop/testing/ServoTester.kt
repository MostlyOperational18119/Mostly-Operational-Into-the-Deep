package org.firstinspires.ftc.teamcode.teleop.testing

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.ServoImplEx

@TeleOp(name = "ServoPositionTester", group = "Basic Chassis")
class ServoTester : LinearOpMode() {
    override fun runOpMode() {

        val transferServo = hardwareMap.servo["Transfer"]
        val outClawServo = hardwareMap.servo["OutClaw"]
        val outRotationServo = hardwareMap.servo["OutRotation"]
        val outSwivelServo = hardwareMap.servo["OutSwivel"]
        val inRotationServo = hardwareMap.servo["InRotation"]
        val inStopServo = hardwareMap.servo["InStop"]
        val intakeMotor = hardwareMap.dcMotor["intakeMotor"]
        var intakeRunPower = 0.3

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
                outRotationServo.position += 0.01
            } else if (currentGamepad1.b && !previousGamepad1.b) {
                outRotationServo.position -= 0.01
            }

            if (currentGamepad1.x && !previousGamepad1.x) {
                inRotationServo.position += 0.01
            } else if (currentGamepad1.y && !previousGamepad1.y) {
                inRotationServo.position -= 0.01
            }

            if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right) {
                intakeRunPower += 0.05
            }
            if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left) {
                intakeRunPower -= 0.05
            }
            if (currentGamepad1.dpad_up && !previousGamepad1.dpad_up) {
                intakeMotor!!.power = intakeRunPower
            }
            if (currentGamepad1.dpad_down && !previousGamepad1.dpad_down) {
                intakeMotor!!.power = 0.0
            }

            telemetry.addData("transferServo position:", transferServo.position)
            telemetry.addData("outClaw position:", outClawServo.position)
            telemetry.addData("outRotation (AB) position:", outRotationServo.position)
            telemetry.addData("outSwivel position:", outSwivelServo.position)
            telemetry.addData("inRotation (XY) position:", inRotationServo.position)
            telemetry.addData("inStopServo position:", inStopServo.position)
            telemetry.update()
        }
    }
}