package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple

@TeleOp(name = "SampleTeleOp", group = "Among Us")
class SampleTeleOp : LinearOpMode() {
    override fun runOpMode () {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val speedDiv = 3.0
        val maxSlide = 4500
        val topSlide = 3800
        var target = 100.0

        val FL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val BL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val FR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val BR = hardwareMap.get(DcMotor::class.java, "motorBR")
        BR.direction = DcMotorSimple.Direction.REVERSE
        FR.direction = DcMotorSimple.Direction.REVERSE

        val rotateMotor = hardwareMap.get(DcMotor::class.java, "motorRotate")
        rotateMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotateMotor.targetPosition = 0
        rotateMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        rotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val slideMotor = hardwareMap.get(DcMotor::class.java, "motorSlide")
        slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideMotor.targetPosition = 0
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION)
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()

        var rotateTarget = 0
        var slideTarget = 0
        while(opModeIsActive()) {
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            val lj = gamepad2.left_stick_y.toDouble()
            val rj = gamepad2.right_stick_y.toDouble()

            FL.power = y + x + rx
            BL.power = y - x + rx
            FR.power = y - x - rx
            BR.power = y + x - rx

            if (rj>0) {
                rotateMotor.targetPosition = -200
                rotateMotor.power = rj*2
                rotateTarget = 0
//                rotationMotor.
            } else if (rj<0) {
                rotateMotor.targetPosition = 900
                rotateMotor.power = -rj*2
                rotateTarget = 0
            } else {
                if (rotateTarget == 0) {
                    rotateMotor.targetPosition = rotateMotor.currentPosition
                    rotateTarget = rotateMotor.currentPosition
                }
            }


            if (lj>0) {
                slideMotor.targetPosition = 0
                slideMotor.power = lj/2
                slideTarget = 0
            } else if (lj<0) {
                rotateMotor.targetPosition = 3800
                rotateMotor.power = -lj/2
                slideTarget = 0
            } else {
                if (slideTarget == 0) {
                    rotateMotor.targetPosition = rotateMotor.currentPosition
                    slideTarget = rotateMotor.currentPosition
                }
            }


            telemetry.addData("BL Power: ",  BL.power)
            telemetry.addData("BR Power:  ",  BR.power)
            telemetry.addData("FL Power: ",  FL.power)
            telemetry.addData("FR Power:  ",  FR.power)
            telemetry.addData("Slide Encoder Position: ",  slideMotor?.let { (it.currentPosition)})
            telemetry.addData("Rotate Encoder Position: ",  rotateMotor?.let { (it.currentPosition)})
            telemetry.addData("Rotate Power: ",  rotateMotor?.let { (it.power)})
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}