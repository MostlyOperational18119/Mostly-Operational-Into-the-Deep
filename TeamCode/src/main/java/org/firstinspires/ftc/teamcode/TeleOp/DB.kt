package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "DB", group = "Among Us")
class DB : LinearOpMode(){
    override fun runOpMode() {
        val right = hardwareMap.get(DcMotor::class.java, "rightwheel")
        val left = hardwareMap.get(DcMotor::class.java, "leftwheel")
        val rotate = hardwareMap.get(DcMotor::class.java, "motorRotate")
        val launcher = hardwareMap.get(Servo::class.java, "launchservo")
        val clawRotate = hardwareMap.get(Servo::class.java, "clawrotation")
        val claw = hardwareMap.get(Servo::class.java, "claw")

        val speedDiv = 3.0

        val startPos = .63
        val stopPos = .9

        val clawOpen = 0.3
        val clawClose = 0.55

        var contPow = 0.0

        launcher.position = startPos
        claw.position = clawOpen
        clawRotate.position = 0.25

        waitForStart()
        while(opModeIsActive()) {
            val leftY = gamepad1.left_stick_y.toDouble()
            val rightx = gamepad1.right_stick_x.toDouble()

            right.power = (leftY - rightx) / speedDiv
            left.power = (leftY + rightx) / speedDiv

            if(gamepad1.a) { launcher.position = startPos }
            if(gamepad1.y) { launcher.position = stopPos }

            if(gamepad1.x) { claw.position = clawOpen }
            if(gamepad1.b) { claw.position = clawClose }

            if(gamepad1.dpad_up) { rotate.power = -.2 }
            else if(gamepad1.dpad_down) { rotate.power = .2 }
            else { rotate.power = 0.0 }

            if (gamepad1.left_bumper && clawRotate.position < 0.5) {clawRotate.position += 0.01
                sleep(33)}
            if (gamepad1.right_bumper && clawRotate.position > 0.16) {clawRotate.position -= 0.01
            sleep(33)}

            telemetry.addData("clawrotate", clawRotate.position)
            telemetry.update()
        }
    }
}