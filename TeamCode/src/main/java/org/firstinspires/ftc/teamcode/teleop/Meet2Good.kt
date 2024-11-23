package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.teleop.Meet2TeleOp.Companion
import kotlin.math.abs
import kotlin.math.exp

@TeleOp(name = "Meet2Good")
class Meet2Good :LinearOpMode() {
    //Function that Initializes all the motors
    override fun runOpMode() {
        fun setMotorModeEncoder(motor: DcMotor) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.power = 0.0
        }

        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val hangerMotor = hardwareMap.dcMotor["hanger"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        val VerticalSlideLow = 1750
        val VerticalSlideHigh = 3790

        val MagicEquationSpeed = 1000
        val MagicEquationMax = 1
        var Target = 0

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

        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        //clawRotateServo.position = 0.0
        waitForStart()
        while (opModeIsActive()) {
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

            if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                Target = 100
            }
            if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left){
                Target = 1000
            }
            if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                Target = 3000
            }

            if (slideVerticalMotor.currentPosition <= Target) {
                slideVerticalMotor.power = ((2 / (1 + (exp(-(Target - slideVerticalMotor.currentPosition).toDouble() / MagicEquationSpeed)))) - 1) * (MagicEquationMax)
            } else {
                slideVerticalMotor.power = -0.05
            }

            telemetry.addLine(slideVerticalMotor.currentPosition.toString())
            telemetry.update()
        }
    }
};
