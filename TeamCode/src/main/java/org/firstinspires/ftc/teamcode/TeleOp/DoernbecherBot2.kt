package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "DoernbecherTeleop", group="Basic Chassis")
class DoernbecherTeleOp: LinearOpMode() {
    override fun runOpMode() {
        val motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        //reverse FR
        val motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        motorFR.direction=DcMotorSimple.Direction.REVERSE
        val motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")
        val aimMotor = hardwareMap.get(DcMotor::class.java, "aimMotor")
        val airplaneServo = hardwareMap.get(Servo::class.java, "airplaneServo")
        val lowerServoPosition = 0.7
        val highestServoPosition = 1.0
        val speedDiv = 2.0

        telemetry.addData("Status", "Initialized")
        telemetry.update()
        waitForStart()
        while (opModeIsActive()) {
            //leftX is intended for strafing
            val leftX = gamepad1.left_stick_x
            //leftY is intended for forward and backward
            val leftY = -gamepad1.left_stick_y
            //rigthX is indended for rotation
            val rightX = gamepad1.right_stick_x

            motorFL.power = (leftY + leftX + rightX) / speedDiv
            motorBL.power = (leftY - leftX + rightX) / speedDiv
            motorFR.power = (leftY - leftX - rightX) / speedDiv
            motorBR.power = (leftY + leftX - rightX) / speedDiv

            telemetry.addData("Status", "Running")

            telemetry.addLine("motorFL: ${motorFL.power} motorFR: ${motorFR.power} motorBL: ${motorBL.power} motorBR: ${motorBR.power} ")
            if (gamepad1.dpad_down && aimMotor.currentPosition<300) {
                telemetry.addLine("Running AIM Motor Original Position: ${aimMotor.currentPosition}")
                aimMotor.power = 0.1
                sleep(80)
                aimMotor.power = 0.0
                telemetry.addLine("Running AIM Motor New Position: ${aimMotor.currentPosition}")
            }else if (gamepad1.dpad_up && aimMotor.currentPosition>-250) {
                telemetry.addLine("Running AIM Motor Original Position: ${aimMotor.currentPosition}")
                aimMotor.power =- 0.2
                sleep(80)
                aimMotor.power = 0.0
                telemetry.addLine("Running AIM Motor New Position: ${aimMotor.currentPosition}")
            }

            if(gamepad1.a){
                airplaneServo.position = lowerServoPosition
            } else if (gamepad1.b){
                airplaneServo.position = highestServoPosition
            }


            telemetry.update()
            sleep(10)
        }
    }
}
