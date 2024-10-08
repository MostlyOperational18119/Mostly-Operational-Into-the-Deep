package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "SampleTeleOp", group = "Among Us")
class SampleTeleOp : LinearOpMode() {
    override fun runOpMode () {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        //VARIABLES
        var rotateTarget = 0
        var slideTarget = 0
        val speedDiv = 2
        val rotateServoMid = 0.2
        val rotateServoLong = 0.4
        val rotateServoShort = 0.0

        //TOGGLES
        var rightintakeToggle = false
        var leftintakeToggle = false

        //GAME PADS
        val currentGamepad1: Gamepad = Gamepad()
        val currentGamepad2: Gamepad = Gamepad()
        val previousGamepad1: Gamepad = Gamepad()
        val previousGamepad2: Gamepad = Gamepad()

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

        val clawServo = hardwareMap.get(CRServo::class.java, "clawServo")
        val rotateServo = hardwareMap.get(Servo::class.java, "rotateServo")


        waitForStart()

        while(opModeIsActive()) {
            //GAMEPADS
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);


            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            val lj = gamepad2.left_stick_y.toDouble()
            val rj = gamepad2.right_stick_y.toDouble()

            //MOVEMENT
            FL.power = (y + x + rx)/speedDiv
            BL.power = (y - x + rx)/speedDiv
            FR.power = (y - x - rx)/speedDiv
            BR.power = (y + x - rx)/speedDiv

            //CLAW SERVO
            if (currentGamepad2.right_bumper&& !previousGamepad2.right_bumper){
                rightintakeToggle = !rightintakeToggle
                leftintakeToggle = false
            }
            if (currentGamepad2.left_bumper&& !previousGamepad2.left_bumper){
                leftintakeToggle = !leftintakeToggle
                rightintakeToggle = false
            }

            if (rightintakeToggle) {
                clawServo?.power = 1.0
            }
            else if (leftintakeToggle) {
                clawServo?.power = -1.0
            }
            else { clawServo?.power = 0.0 }

            //ROTATE SERVO
            if (slideMotor.currentPosition < 500){
                rotateServo.position = rotateServoMid
            }
            else if (currentGamepad2.y&& !previousGamepad2.y) {
                rotateServo.position = rotateServoLong
            }
            else if (currentGamepad2.a&& !previousGamepad2.a) {
                rotateServo.position = rotateServoShort
            }
            else if (currentGamepad2.b&& !previousGamepad2.b){
                rotateServo.position = rotateServoMid
            }


            //ROTATE
            if (rj>0) {
                rotateMotor.targetPosition = -200
                rotateMotor.power = rj/3
                rotateTarget = 0
            } else if (rj<0) {
                rotateMotor.targetPosition = 1250
                rotateMotor.power = -rj/3
                rotateTarget = 0
            } else {
                if (rotateTarget == 0) {
                    rotateMotor.targetPosition = rotateMotor.currentPosition
                    rotateTarget = rotateMotor.currentPosition
                }
            }

            //SLIDES
            if (lj>0) {
                slideMotor.targetPosition = 0
                slideMotor.power = lj/2
                slideTarget = 0
            } else if (lj<0) {
                slideMotor.targetPosition = 4000
                slideMotor.power = -lj/2
                slideTarget = 0
            } else {
                if (slideTarget == 0) {
                    slideMotor.targetPosition = slideMotor.currentPosition
                    slideTarget = slideMotor.currentPosition
                }
            }

            //TELEMETRY
            telemetry.addData("BL Power: ",  BL.power)
            telemetry.addData("BR Power:  ",  BR.power)
            telemetry.addData("FL Power: ",  FL.power)
            telemetry.addData("FR Power:  ",  FR.power)
            telemetry.addData("RotateServo Postion:  ",  rotateServo.position)
            telemetry.addData("Slide Encoder Position: ",  slideMotor?.let { (it.currentPosition)})
            telemetry.addData("Rotate Encoder Position: ",  rotateMotor?.let { (it.currentPosition)})
            telemetry.addData("Rotate Power: ",  rotateMotor?.let { (it.power)})
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}