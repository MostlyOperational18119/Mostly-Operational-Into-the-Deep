package org.firstinspires.ftc.teamcode.TeleOp

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.*

@TeleOp(name = "RohanProject", group = "Z")
@Disabled
class RohanJamesProject : LinearOpMode() {
    override fun runOpMode () {

        telemetry.addData("Status", "Initialized")
        telemetry.update()

        val speedDiv = 2.0

        //MATH STUFF
        var posX = 0.0
        var posY = 0.0
        var angle = 7*PI/18
        var length = 49.5 //in
        var maxLength = 49.5
        val horizontalEncoder = 900
        val behindEncoder = -200.0
        val topEncoder = 3800
        val slideSlope = topEncoder/length

        //TOGGLES
        var coordinator = true

        //GAMEPADS
        val currentGamepad1: Gamepad = Gamepad()
        val currentGamepad2: Gamepad = Gamepad()
        val previousGamepad1: Gamepad = Gamepad()
        val previousGamepad2: Gamepad = Gamepad()

        //MOTORS
        val motorFL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val motorBL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val motorFR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val motorBR = hardwareMap.get(DcMotor::class.java, "motorBR")
        motorBR.direction = DcMotorSimple.Direction.REVERSE
        motorFR.direction = DcMotorSimple.Direction.REVERSE

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

        //SERVOS
        val clawServo = hardwareMap.get(CRServo::class.java, "clawServo")

        waitForStart()

        while(opModeIsActive()) {
            previousGamepad1.copy(currentGamepad1);
            previousGamepad2.copy(currentGamepad2);
            currentGamepad1.copy(gamepad1);
            currentGamepad2.copy(gamepad2);

            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            val lj = -gamepad2.left_stick_y.toDouble()
            val rj = -gamepad2.right_stick_y.toDouble()

            motorFL.power = y + x + rx
            motorBL.power = y - x + rx
            motorFR.power = y - x - rx
            motorBR.power = y + x - rx

            posX += (lj/10)
            posY += (rj/10)

            if (currentGamepad2.x&& !previousGamepad2.x){
                clawServo?.power = 1.0
            }
            if (currentGamepad2.y&& !previousGamepad2.y){
                clawServo?.power = 0.0
            }

            if (currentGamepad1.a && !previousGamepad1.a){
                // CALCULATE VALUES
                    length = hypot(posX,posY)   // Find length of slide needed in inches
                    length *= slideSlope
                    angle = atan2(posX,posY)  // Find Angle in Radians
                    angle = angle * 180.0/PI //Now its in Degrees
                    angle = horizontalEncoder - angle * 10   // Now its in encoders

                //CHECK IF EVERYTHING IS FINE
                    if (length > maxLength|| length < 12.0){
                        coordinator = false
                    }
                    if (angle < behindEncoder|| angle > horizontalEncoder) {
                        coordinator = false
                    }
                    if (posX > 24 || posX < 0){
                        coordinator = false
                    }
                    if (posY > 49.5|| posY < 12){
                        coordinator = false
                    }


                //MOVE STUFF
//                if (coordinator) {
//                    rotateMotor.targetPosition = angle.toInt()
//                    rotateMotor.power = 0.1
//
//                    slideMotor.targetPosition = length.toInt()
//                    slideMotor.power = 0.1
//                }

                coordinator = true
            }

            telemetry.addData("X: ", posX)
            telemetry.addData("Y: ", posY)
            telemetry.addData("Rotate Target: ", rotateMotor.targetPosition )
            telemetry.addData("Slide Target: ", slideMotor.targetPosition )
            telemetry.addData("Angle: ", angle )
            telemetry.addData("Length: ", length )
            telemetry.addData("Coordinator: ", coordinator)
            telemetry.addData("Servo Power: ", clawServo.power )
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}