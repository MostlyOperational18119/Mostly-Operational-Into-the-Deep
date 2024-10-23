@file:Suppress("PackageName")

package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "MotorTester", group = "0000000")
@Suppress("Unused")
class MotorTester : LinearOpMode() {
    override fun runOpMode () {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")
        motorBR.direction = DcMotorSimple.Direction.REVERSE
        motorFR.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()
        while(opModeIsActive()) {
            var leftX = gamepad1.left_stick_x.toDouble()
            var leftY = -gamepad1.left_stick_y.toDouble()
            var rightX = gamepad1.right_stick_x.toDouble()

            motorFL?.power = (leftY + leftX + rightX)
            motorBL?.power = (leftY - leftX + rightX)
            motorFR?.power = (leftY - leftX - rightX)
            motorBR?.power = (leftY + leftX - rightX)


            telemetry.addData("BL Power: ",  motorBL.power)
            telemetry.addData("BR Power:  ", motorBR.power)
            telemetry.addData("FL Power: ",  motorFL.power)
            telemetry.addData("FR Power:  ",  motorFR.power)
            telemetry.addData("Left Stick Y:  ",  gamepad1.left_stick_y)
            telemetry.addData("Left Stick X:  ",  gamepad1.left_stick_x)
            telemetry.addData("Right Stick X:  ",  gamepad1.right_stick_x)
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}