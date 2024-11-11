package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo

@TeleOp(name = "DB_SMALL", group = "AAAAAAA")
class  cccccDB_Small : LinearOpMode(){
    override fun runOpMode() {
        val right = hardwareMap.get(DcMotor::class.java, "rightwheel")
        val left = hardwareMap.get(DcMotor::class.java, "leftwheel")
        val rotate = hardwareMap.get(DcMotor::class.java, "motorRotate")
        val launcher = hardwareMap.get(Servo::class.java, "launchservo")
        val clawRotate = hardwareMap.get(Servo::class.java, "clawrotation")
        val claw = hardwareMap.get(Servo::class.java, "claw")

        //VARIABLES
        val speedDiv = 3.0
        val startPos = .63
        val stopPos = .9
        val clawOpen = 0.3
        val clawClose = 0.55
        var contPow = 0.0

        //TOGGLES
        var leftArmToggle = false
        var rightArmToggle = false

        //GAME PADS
        val currentGamepad1: Gamepad = Gamepad()
        val currentGamepad2: Gamepad = Gamepad()
        val previousGamepad1: Gamepad = Gamepad()
        val previousGamepad2: Gamepad = Gamepad()

        launcher.position = startPos
        claw.position = clawOpen
        clawRotate.position = 0.25
        rotate.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        rotate.targetPosition = 0
        rotate.power = 0.2
        rotate.mode = DcMotor.RunMode.RUN_TO_POSITION
        rotate.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        waitForStart()
        while(opModeIsActive()) {
            //GAMEPADS
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            //INPUTS
            val leftY = gamepad1.left_stick_y.toDouble()
            val rightx = gamepad1.right_stick_x.toDouble()

            //MOVEMENT
            right.power = (leftY - rightx) / speedDiv
            left.power = (leftY + rightx) / speedDiv

            //Reset Purple
            if (currentGamepad1.b&& !previousGamepad1.b){ leftArmToggle = !leftArmToggle }
            if (currentGamepad1.x&& !previousGamepad1.x){ rightArmToggle = !rightArmToggle }

            //MOVE SERVOS
            if(!leftArmToggle) { launcher.position = startPos }  //Right Reset
            if(leftArmToggle) { launcher.position = stopPos }    //Right Launch
            if(rightArmToggle) { claw.position = clawOpen }      //Left Up
            if(!rightArmToggle) { claw.position = clawClose }    //Left Down

            //ROTATION
            if (gamepad1.dpad_down && rotate.targetPosition < -10) {
                rotate.targetPosition += 10
                sleep(33)
            }
            if (gamepad1.dpad_up && rotate.targetPosition > -630) {
                rotate.targetPosition -= 10
                sleep(33)
            }
            if (gamepad1.left_bumper && clawRotate.position < 0.45) {
                clawRotate.position += 0.01
                sleep(33)
            }
            if (gamepad1.right_bumper && clawRotate.position > 0.11) {
                clawRotate.position -= 0.01
                sleep(33)
            }

            //TELEMETRY
            telemetry.addData("clawrotate", clawRotate.position)
            telemetry.addData("rotate", rotate.targetPosition)
            telemetry.update()
        }
    }
}