package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs
import kotlin.math.exp

@TeleOp(name = "Meet2TeleOp")
class Meet2TeleOp : LinearOpMode() {
    //Function that Initializes all the motors
    fun setMotorModeEncoder(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    //Function that does all the power control
    fun powerControl(motor: DcMotor, target: Int) {
        //If the equation gives a power too low, just set it to 0.3
        if (abs(-((2 / (1 + (exp(-(target - motor.currentPosition).toDouble() / MagicEquationSpeed)))) - 1) * (MagicEquationMax)) < 0.3){
            motor.power = 0.3
        }
        else {
            motor.power = -((2 / (1 + (exp(-(target - motor.currentPosition).toDouble() / MagicEquationSpeed)))) - 1) * (MagicEquationMax)
        }
    }

    override fun runOpMode() {
        waitForStart()

        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val hangerMotor = hardwareMap.dcMotor["hanger"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        // Set motor modes
        setMotorModeEncoder(slideVerticalMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(hangerMotor)
//        setMotorModeEncoder(grabberExtensionMotor)
//        setMotorModeEncoder(tapeMeasureRotateMotor)

        // Servos
        //val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        val clawRotateServo = hardwareMap.servo["clawRotate"]
        val hangPusher = hardwareMap.servo["hangPusher"] // Linear servo

        // Limit switches
        //val limitSwitchL = hardwareMap.touchSensor["limitSwitchL"]
        //val limitSwitchR = hardwareMap.touchSensor["limitSwitchR"]

        //Odometry Stuff
        val drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = PoseStorage.currentPose
        val redBoxVector = Vector2d(-58.26, -57.64)
        val redBoxHeading = Math.toRadians(225.00)
        val blueBoxVector = Vector2d(56.99, 57.75)
        val blueBoxHeading = Math.toRadians(45.00)

        // Speed
        val speedDiv = 1.5

        // Gamepads
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        // Fun toggles
        var verticalSlideToggle = VerticalSlideTarget.Manual

        while (opModeIsActive()) {
            // Update Gamepads :)
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            // Get input values
            val leftY = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val leftX = gamepad1.left_stick_x.toDouble()
            val rightX = gamepad1.right_stick_x.toDouble()
            val leftY2 = -gamepad2.left_stick_y.toDouble()
            val rightY2 = -gamepad2.right_stick_y.toDouble()

            // Move robot
            motorFL.power = (leftY + leftX + rightX) / speedDiv
            motorBL.power = (leftY - leftX + rightX) / speedDiv
            motorFR.power = (leftY - leftX - rightX) / speedDiv
            motorBR.power = (leftY + leftX - rightX) / speedDiv

            // Slide
            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                verticalSlideToggle = VerticalSlideTarget.Floor
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
                verticalSlideToggle = VerticalSlideTarget.Low
            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                verticalSlideToggle = VerticalSlideTarget.High
            }
            if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right){
                verticalSlideToggle = VerticalSlideTarget.Manual
            }

            when (verticalSlideToggle) {
                VerticalSlideTarget.Manual -> {
                    slideVerticalMotor.power = (leftY2/3)
                    slideVerticalMotor.currentPosition
                }

                VerticalSlideTarget.Floor -> {
                    slideVerticalMotor.targetPosition = VerticalSlideFloor
                    powerControl(slideVerticalMotor, VerticalSlideFloor)
                    if (slideVerticalMotor.currentPosition == VerticalSlideFloor){
                        verticalSlideToggle = VerticalSlideTarget.Manual
                    }
                }
                VerticalSlideTarget.Low -> {
                    slideVerticalMotor.targetPosition = VerticalSlideLow
                    powerControl(slideVerticalMotor, VerticalSlideLow)
                    if (slideVerticalMotor.currentPosition == VerticalSlideLow){
                        verticalSlideToggle = VerticalSlideTarget.Manual
                    }
                }

                VerticalSlideTarget.High -> {
                    slideVerticalMotor.targetPosition = VerticalSlideHigh
                    powerControl(slideVerticalMotor, VerticalSlideHigh)
                    if (slideVerticalMotor.currentPosition == VerticalSlideHigh){
                        verticalSlideToggle = VerticalSlideTarget.Manual
                    }
                }
            }

            //Horizontal Motor
            slideHorizontalMotor.targetPosition += rightY2.toInt()*5
            slideHorizontalMotor.power = 0.3


            // Telemetry
//            telemetry.addData("MotorBL Power: ", motorBL.power)
//            telemetry.addData("Motor BR Power:  ", motorBR.power)
//            telemetry.addData("Motor FL Power: ", motorFL.power)
//            telemetry.addData("Motor FR Power:  ", motorFR.power)
            telemetry.addData("Vertical Slide Power: ", slideVerticalMotor.power)
            telemetry.addData("Vertical Slide Target: ", slideVerticalMotor.targetPosition)
            telemetry.addData("Vertical Slide Position: ", slideVerticalMotor.currentPosition)
            telemetry.addData("Horizontal Slide Power: ", slideHorizontalMotor.power)
            telemetry.addData("Horizontal Slide Target: ", slideHorizontalMotor.targetPosition)
            telemetry.addData("Horizontal Slide Position: ", slideHorizontalMotor.currentPosition)
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }

    companion object {
        const val HorizontalSlideInClicks = 0
        const val HorizontalSlideOutClicks = 0 // TODO: find correct value

        const val VerticalSlideFloor = 0
        const val VerticalSlideLow = 1750
        const val VerticalSlideHigh = 3790

        const val MagicEquationSpeed = 2000
        const val MagicEquationMax = 0.1
        // Magic linear slide equation: -((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max
    }
}

enum class VerticalSlideTarget {
    Floor,
    Low,
    High,
    Manual
}

enum class DRIVE_STATE{
    MANUAL,
    AUTONOMOUS
};
