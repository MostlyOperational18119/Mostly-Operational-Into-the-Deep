package org.firstinspires.ftc.teamcode.teleop.outreach

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "DB_BIG", group = "AAAAAAAA")
class DoernbecherTeleOp : LinearOpMode() {
    override fun runOpMode() {
        val motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        val motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")

        val aimMotor = hardwareMap.get(DcMotor::class.java, "aimMotor")
        aimMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        val airplaneServo = hardwareMap.get(Servo::class.java, "airplaneServo")

        val lowerServoPosition = 0.7
        val highestServoPosition = 1.0
        val speedDiv = 2.0
        var armToggle = false

        val currentGamepad1: Gamepad = Gamepad()
        val currentGamepad2: Gamepad = Gamepad()
        val previousGamepad1: Gamepad = Gamepad()
        val previousGamepad2: Gamepad = Gamepad()

        telemetry.addData("Status", "Initialized")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            //GAMEPADS
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            //INPUT
            val leftX = gamepad1.left_stick_x     //STRAFING
            val leftY = -gamepad1.left_stick_y    //FORWARD + BACK
            val rightX = gamepad1.right_stick_x   //ROTATE

            //MOVE
            motorFL.power = (leftY + leftX + rightX) / speedDiv
            motorBL.power = (leftY - leftX + rightX) / speedDiv
            motorFR.power = (leftY - leftX - rightX) / speedDiv
            motorBR.power = (leftY + leftX - rightX) / speedDiv

            //AIM MOTOR
            if (currentGamepad1.dpad_down) {
                aimMotor.power = 0.1
                sleep(80)
                aimMotor.power = 0.0
            } else if (currentGamepad1.dpad_up) {
                aimMotor.power = -0.2
                sleep(80)
                aimMotor.power = 0.0
            }

            //TOGGLES ON/OFF
            if (currentGamepad1.b && !previousGamepad1.b) {
                armToggle = !armToggle
            }

            //AIRPLANE SERVO
            if (armToggle) {
                airplaneServo.position = lowerServoPosition
            }
            if (!armToggle) {
                airplaneServo.position = highestServoPosition
            }

            //TELEMETRY
            telemetry.addData("Status", "Running")
            telemetry.addLine("motorFL: ${motorFL.power} motorFR: ${motorFR.power} motorBL: ${motorBL.power} motorBR: ${motorBR.power} ")
            telemetry.addLine("AIM Motor Position: ${aimMotor.currentPosition}")
            telemetry.update()
            sleep(10)
        }
    }
}
