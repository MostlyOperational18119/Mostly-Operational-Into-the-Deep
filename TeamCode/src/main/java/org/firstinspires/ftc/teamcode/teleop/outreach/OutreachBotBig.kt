package org.firstinspires.ftc.teamcode.teleop.outreach

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad

@TeleOp(name = "OutreachBotBig")
class OutreachBotBig : LinearOpMode() {
    override fun runOpMode() {
        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]

        // Reverse right motors
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        motorBR.direction = DcMotorSimple.Direction.REVERSE

        // Gamepads
        var currentGamepad1 = Gamepad()
        var previousGamepad1 = Gamepad()

        // Other variables
        val speedDiv = 2.0

        telemetry.addLine("Init done :)")
        telemetry.update()

        waitForStart()

        while(opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1)
            currentGamepad1.copy(gamepad1)

            val gamepad1LeftY = -currentGamepad1.left_stick_y
            val gamepadLeftX = currentGamepad1.left_stick_x
            val gamepad1RightX = currentGamepad1.right_stick_x

            motorFL.power = (gamepad1LeftY + gamepadLeftX + gamepad1RightX) / speedDiv
            motorBL.power = (gamepad1LeftY - gamepadLeftX + gamepad1RightX) / speedDiv
            motorFR.power = (gamepad1LeftY - gamepadLeftX - gamepad1RightX) / speedDiv
            motorBR.power = (gamepad1LeftY + gamepadLeftX - gamepad1RightX) / speedDiv

            sleep(50)
        }
    }
}