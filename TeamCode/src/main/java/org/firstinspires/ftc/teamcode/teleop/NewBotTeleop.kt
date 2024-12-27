package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "The Brainiacs \uD83E\uDDE0", group = "ZZZ")
class NewBotTeleop: LinearOpMode() {
    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var slideVertical: DcMotor? = null
    var slideHorizontal: DcMotor? = null

    var leftY1: Double? = null
    var leftX1: Double? = null
    var rightX1: Double? = null
    var leftY2: Double? = null
    var rightY2: Double? = null

    override fun runOpMode() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVertical = hardwareMap.dcMotor["verticalSlide"]
        slideHorizontal = hardwareMap.dcMotor["horizontalSlide"]
        slideHorizontal!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideHorizontal!!.targetPosition = 0
        slideHorizontal!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideHorizontal!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideVertical!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideVertical!!.targetPosition = 0
        slideVertical!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideVertical!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorBL!!.direction = DcMotorSimple.Direction.REVERSE
        motorFL!!.direction = DcMotorSimple.Direction.REVERSE
        slideHorizontal!!.direction = DcMotorSimple.Direction.REVERSE

        waitForStart()

        while (opModeIsActive()) {
            leftY1 = -gamepad1.left_stick_y.toDouble()
            leftX1 = gamepad1.left_stick_x.toDouble()
            rightX1 = gamepad1.right_stick_x.toDouble()
            leftY2 = -gamepad2.left_stick_y.toDouble()
            rightY2 = -gamepad2.right_stick_y.toDouble()

            motorFL!!.power = (leftY1!! + leftX1!!+ rightX1!!)
            motorBL!!.power = (leftY1!! - leftX1!! + rightX1!!)
            motorFR!!.power = (leftY1!! - leftX1!! - rightX1!!)
            motorBR!!.power = (leftY1!! + leftX1!! - rightX1!!)

            if (leftY2!! > 0 && slideHorizontal!!.currentPosition < 1000) {
                slideHorizontal!!.targetPosition = 1000
                slideHorizontal!!.power = leftY2 as Double
            }
            else if (leftY2!! < 0 && slideHorizontal!!.currentPosition > 0) {
                slideHorizontal!!.targetPosition = 0
                slideHorizontal!!.power = leftY2 as Double
            }
            else {
                slideHorizontal!!.targetPosition = slideHorizontal!!.currentPosition
                slideHorizontal!!.power = 0.1
            }

            if (rightY2!! > 0 && slideVertical!!.currentPosition < 4000) {
                slideVertical!!.targetPosition = 4000
                slideVertical!!.power = (rightY2 as Double)/3
            }
            else if (rightY2!! < 0 && slideVertical!!.currentPosition > 0) {
                slideVertical!!.targetPosition = 0
                slideVertical!!.power = (rightY2 as Double)/3
            }
            else {
                slideVertical!!.targetPosition = slideVertical!!.currentPosition
                slideVertical!!.power = 0.1
            }


            telemetry.addData("Vertical Current: ", slideVertical!!.currentPosition)
            telemetry.addData("Vertical Target: ", slideVertical!!.currentPosition)
            telemetry.addLine(slideHorizontal!!.targetPosition.toString())
            telemetry.update()
        }
    }
}