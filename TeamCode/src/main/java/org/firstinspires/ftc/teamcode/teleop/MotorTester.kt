package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "MotorTester", group = "ZZZZZZZZZ")
class MotorTester : LinearOpMode() {
    override fun runOpMode () {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val FL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val BL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val FR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val BR = hardwareMap.get(DcMotor::class.java, "motorBR")
        BR.direction = DcMotorSimple.Direction.REVERSE
        FR.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()
        while(opModeIsActive()) {
            var leftX = gamepad1.left_stick_x.toDouble()
            var leftY = -gamepad1.left_stick_y.toDouble()
            var rightX = gamepad1.right_stick_x.toDouble()

            FL?.power = (leftY + leftX + rightX)
            BL?.power = (leftY - leftX + rightX)
            FR?.power = (leftY - leftX - rightX)
            BR?.power = (leftY + leftX - rightX)


            telemetry.addData("BL Power: ",  BL.power)
            telemetry.addData("BR Power:  ",  BR.power)
            telemetry.addData("FL Power: ",  FL.power)
            telemetry.addData("FR Power:  ",  FR.power)
            telemetry.addData("Left Stick Y:  ",  gamepad1.left_stick_y)
            telemetry.addData("Left Stick X:  ",  gamepad1.left_stick_x)
            telemetry.addData("Right Stick X:  ",  gamepad1.right_stick_x)
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}