package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs
import kotlin.math.exp

@TeleOp(name = "Meet2Good")
class Meet2Good :LinearOpMode() {
    enum class verticalSlideState {
        Floor,
        Low,
        High,
        Manual
    }
    enum class horizontalSlideState {
        Floor,
        Extend,
        Manual
    }
    enum class automaticTransferState {
        Manual,
        HorizontalFloor
    }

    //Function that Initializes all the motors
    override fun runOpMode() {
        telemetry.addLine(when ((0..49).random()) {
            1 -> "good luck buddy"
            2 -> "\"what spectrum?\""
            3 -> "MostlyOp >>> AHoT"
            4 -> "01101011 01101001 01101100 01101100 00100000 01111001 01101111 01110101 01110010 01110011 01100101 01101100 01100110"
            5 -> "I LOVE ULTRAKILL!!!!!!!!!!!!"
            6 -> "\"just hit clean build\""
            7 -> "this match is gonna be ghoulish green"
            8 -> "we are so back"
            9 -> "ok so would you rather have a 1% chance of becoming a turkey everyday or..."
            10 -> "RIP damien mode 2022-2023"
            11 -> "build freeze at 3 AM challenge (GONE WRONG)"
            12 -> "\"who unqueued my song?\""
            13 -> "at least we don't have a pushbot! (not confirmed, high likelyhood of pushbot)"
            14 -> "whoever set continuous rotation as the default is my #1 opp"
            15 -> "shoutout to Huy for being our insider <3"
            16 -> "why does jack always come to TR3? Is he stupid?"
            17 -> "Nick, I need you to sand this."
            18 -> "I wish fame and good fortune upon Sachal's bloodline"
            19 -> "-((2 / (1 + (exp(-(target - Pos) / speed)))) - 1) * max"
            20 -> "\"the grid system is stupid.\" *starts pointing at poles*"
            21 -> "James, how many orange cups have you eaten today?"
            22 -> "Tennisball is the newest sport sweeping across the nation!"
            23 -> "our robot has been too big for the bounding box on 3 different occasions."
            24 -> "cord control is not real"
            25 -> "in Raytheon we trust"
            26 -> "drive practice is for nerds."
            27 -> "Sebastian (yum)"
            28 -> "this is the abyss of our hero's journey."
            29 -> "beware the FTC to Raytheon pipeline"
            30 -> "when build says 15 minutes, expect 30. When programming says 15 minutes, expect 2-60."
            31 -> "99% of programmers quit right before the working push"
            32 -> "Tiger Woods PGA tour 2005 has always been there"
            33 -> "THIS SENT ME \n sent you where? \n TO FINLAND \u1F1E"
            34 -> "How purple?"
            35 -> "That is fragrantly upside down"
            36 -> "I literally just stand here and look at you guys and think god when is this done"
            37 -> "They should be singing in the closet"
            38 -> "autoByJames"
            39 -> "anti-fluent"
            40 -> "fire in the hole"
            41 -> "Antidisestablishmentarianism is a position that advocates that a state church (the \"established church\") should continue to receive government patronage, rather than be disestablished (i.e., be separated from the state)."
            42 -> "#include <iostream>\n\nint main() {\n\tstd::cout << \"Hello World!\\n\";\n\treturn 0;\n}\n"
            43 -> "fn main() {\n\tprintln!(\"Hello World!\");\n}\n"
            44 -> "console.log(\"Hello World!\");"
            45 -> "package main\n\nimport \"fmt\"\n\nfunc main() {\n\tfmt.Println(\"Hello World!\")\n}\n"
            46 -> "with Text_IO; use Text_IO;\nprocedure hello is\nbegin\n\tPut_Line(\"Hello world!\");\nend hello;\n"
            47 -> ">++++++++[<+++++++++>-]<.>++++[<+++++++>-]<+.+++++++..+++.>>++++++[<+++++++>-]<++.------------.>++++++[<+++++++++>-]<+.<.+++.------.--------.>>>++++[<++++++++>-]<+."
            48 -> "main = putStrLn \"Hello, World!\""
            else -> "Why did we add these?"
        })
        telemetry.update()

        fun setMotorModeEncoder(motor: DcMotor) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
            motor.power = 0.0
        }

        fun setMotorModePosition(motor: DcMotor) {
            motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            motor.targetPosition = 0
            motor.mode = DcMotor.RunMode.RUN_TO_POSITION
            motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        }

        // MOTORS
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val hangerMotor = hardwareMap.dcMotor["hanger"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        //MOTORS MODES
        motorBR.direction = DcMotorSimple.Direction.REVERSE
        motorFR.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideVerticalMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(tapeMeasureRotateMotor)
        setMotorModeEncoder(hangerMotor)
        tapeMeasureRotateMotor.targetPosition = 0
//        setMotorModeEncoder(grabberExtensionMotor)
//        setMotorModeEncoder(tapeMeasureRotateMotor)

        // Servos
        val clawRotateRest = 0.72
        val clawRotateUpright = 0.55
        val clawRotateOut = 0.2
        val intakeInPower = 1.0
        val intakeOutPower = -1.0
        val transferDownPos = 0.57
        val transferUpPos = 0.23
        val clawServoOpen = 0.13
        val clawServoClosed = 0.23

        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        val transferServo = hardwareMap.servo["transferServo"]
        val clawRotateServo = hardwareMap.servo["clawRotate"]
        clawRotateServo.position = clawRotateRest
        val hangPusher = hardwareMap.servo["hangPusher"] // Linear servo

        //VARIABLES
        val VerticalSlideFloor = 0
        val VerticalSlideLow = 2300
        val VerticalSlideHigh = 3650

        val HorizontalSlideExtend = 2000

        val MagicEquationSpeed = 2000
        val MagicEquationMax = 0.1
        var speedDiv = 2

        var Target = 0
        var horizontalSlideToggle = horizontalSlideState.Manual
        var verticalSlideToggle = verticalSlideState.Manual
        var automatedTransferToggle = automaticTransferState.Manual
        var intakeInToggle = false
        var intakeOutToggle = false

        //ROADRUNNER
        val drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = Pose2d(0.0,0.0,0.0)//PoseStorage.currentPose
        val BoxVector = Vector2d(-58.26, -57.64)
        val BoxHeading = Math.toRadians(225.00)
        val BoxPose = Pose2d(-58.26, -57.64, 225.0)

        //GAMEPADS
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2 = Gamepad()

        waitForStart()

        while (opModeIsActive()) {
            //GAMEPADS
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            //INPUTS
            val leftY = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val leftX = gamepad1.left_stick_x.toDouble()
            val rightX = gamepad1.right_stick_x.toDouble()
            val leftY2 = -gamepad2.left_stick_y.toDouble()
            val rightY2 = -gamepad2.right_stick_y.toDouble()

            //MOVE
            motorFL.power = (leftY + leftX + rightX) / speedDiv
            motorBL.power = (leftY - leftX + rightX) / speedDiv
            motorFR.power = (leftY - leftX - rightX) / speedDiv
            motorBR.power = (leftY + leftX - rightX) / speedDiv

            //RESET
            if (currentGamepad1.b&& !previousGamepad1.b) {
                setMotorModePosition(slideVerticalMotor)
                setMotorModeEncoder(tapeMeasureRotateMotor)
                setMotorModePosition(slideHorizontalMotor)
                setMotorModeEncoder(hangerMotor)
            }

            //TRANSFER TOGGLES
            if (currentGamepad1.dpad_left&& !previousGamepad1.dpad_left){ automatedTransferToggle = automaticTransferState.HorizontalFloor }

            when (automatedTransferToggle) {
                automaticTransferState.Manual -> {

                    //ROTATESERVO
                    if (gamepad2.right_trigger > 0.5){
                        clawRotateServo.position = clawRotateOut
                    }
                    if (gamepad2.left_trigger > 0.5){
                        clawRotateServo.position = clawRotateRest
                    }
                    if (gamepad2.left_bumper){
                        clawRotateServo.position = clawRotateUpright
                    }

                    //CLAW SERVO
                    if (gamepad1.right_trigger > 0.5){
                        clawServo.position = clawServoClosed
                    }
                    if (gamepad1.left_trigger > 0.5){
                        clawServo.position = clawServoOpen
                    }

                    //INTAKE SERVO
//                    if (currentGamepad1.right_bumper&& !previousGamepad1.right_bumper){
//                        intakeInToggle = !intakeInToggle
//                        intakeOutToggle = false
//                    }
//                    if (currentGamepad1.left_bumper&& !previousGamepad1.left_bumper){
//                        intakeOutToggle = !intakeOutToggle
//                        intakeInToggle = false
//                    }
//                    if (intakeInToggle) { intakeServo?.power = 1.0 }
//                    else if (intakeOutToggle) { intakeServo?.power = -1.0 }
//                    else { intakeServo?.power = 0.0 }

                    //TRANSFER SERVO
                    if (currentGamepad1.right_bumper&& !previousGamepad1.right_bumper){
                        transferServo.position = transferDownPos
                    }
                    if (currentGamepad1.left_bumper&& !previousGamepad1.left_bumper){
                        transferServo.position = transferUpPos
                    }


                    //SLIDES
                    if (currentGamepad2.dpad_down && !previousGamepad2.dpad_down) {
                        verticalSlideToggle = verticalSlideState.Floor
                    }
                    if (currentGamepad2.dpad_left && !previousGamepad2.dpad_left) {
                        verticalSlideToggle = verticalSlideState.Low
                    }
                    if (currentGamepad2.dpad_up && !previousGamepad2.dpad_up) {
                        verticalSlideToggle = verticalSlideState.High
                    }

                    if (currentGamepad2.dpad_right && !previousGamepad2.dpad_right) {
                        verticalSlideToggle = verticalSlideState.Manual
                    }

                    when (verticalSlideToggle) {
                        verticalSlideState.Manual -> {
                            if (rightY2 > 0.0 && slideVerticalMotor.currentPosition < 3650) {
                                slideVerticalMotor.targetPosition = 3650
                                slideVerticalMotor.power = rightY2 / 2
                            } else if (rightY2 < 0.0 && slideVerticalMotor.currentPosition > 0) {
                                slideVerticalMotor.targetPosition = 0
                                slideVerticalMotor.power = rightY2 / 2
                            } else {
                                slideVerticalMotor.targetPosition =
                                    slideVerticalMotor.currentPosition
                                slideVerticalMotor.power = 0.2
                            }
                        }

                        verticalSlideState.Floor -> {
                            slideVerticalMotor.targetPosition = VerticalSlideFloor
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - VerticalSlideFloor) < 20) {
                                verticalSlideToggle = verticalSlideState.Manual
                            }
                        }

                        verticalSlideState.Low -> {
                            slideVerticalMotor.targetPosition = VerticalSlideLow
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - VerticalSlideLow) < 20) {
                                verticalSlideToggle = verticalSlideState.Manual
                            }
                        }

                        verticalSlideState.High -> {
                            slideVerticalMotor.targetPosition = VerticalSlideHigh
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - VerticalSlideHigh) < 20) {
                                verticalSlideToggle = verticalSlideState.Manual
                            }
                        }
                    }

                    //HANGING
                    if (currentGamepad2.a && !previousGamepad2.a) {
                        hangPusher.position = 0.00 //linear position here
                        sleep(3000)
                        hangerMotor.targetPosition = 100 //motor pos here
                    }

//                    if (gamepad1.left_trigger >= 0.2) {
//                        tapeMeasureRotateMotor.power = 1.0
//                    } else if (gamepad1.right_trigger >= 0.2) {
//                        tapeMeasureRotateMotor.power = -1.0
//                    } else {
//                        tapeMeasureRotateMotor.power = 0.0
//                    }


                    //HORIZONTAL MOTOR
                    if (currentGamepad2.y && !previousGamepad2.y) {
                        horizontalSlideToggle = horizontalSlideState.Floor
                    }
                    if (currentGamepad2.x && !previousGamepad2.x) {
                        horizontalSlideToggle = horizontalSlideState.Extend
                    }

                    when (horizontalSlideToggle) {
                        horizontalSlideState.Manual -> {
                            if (leftY2 > 0.0 && slideHorizontalMotor.currentPosition < 2000) {
                                slideHorizontalMotor.targetPosition = 2000
                                slideHorizontalMotor.power = leftY2 / 2
                            } else if (leftY2 < 0.0 && slideHorizontalMotor.currentPosition > 0) {
                                slideHorizontalMotor.targetPosition = 0
                                slideHorizontalMotor.power = leftY2 / 2
                            } else {
                                slideHorizontalMotor.targetPosition =
                                    slideHorizontalMotor.currentPosition
                                slideHorizontalMotor.power = 0.1
                            }
                        }

                        horizontalSlideState.Floor -> {
                            slideHorizontalMotor.targetPosition = 0
                            slideHorizontalMotor.power = -0.8
                            if (abs(slideHorizontalMotor.currentPosition) < 50) {
                                horizontalSlideToggle = horizontalSlideState.Manual
                            }
                        }

                        horizontalSlideState.Extend -> {
                            slideHorizontalMotor.targetPosition = HorizontalSlideExtend
                            slideHorizontalMotor.power = 0.8
                            if (abs(slideVerticalMotor.currentPosition - HorizontalSlideExtend) < 50) {
                                horizontalSlideToggle = horizontalSlideState.Manual
                            }
                        }
                    }
                }
                automaticTransferState.HorizontalFloor ->{

                }
            }

            telemetry.addData("Vertical Slide Power: ", slideVerticalMotor.power)
            telemetry.addData("Vertical Slide Target: ", slideVerticalMotor.targetPosition)
            telemetry.addData("Vertical Slide Position: ", slideVerticalMotor.currentPosition)
            telemetry.addData("Horizontal Slide Power: ", slideHorizontalMotor.power)
            telemetry.addData("Horizontal Slide Target: ", slideHorizontalMotor.targetPosition)
            telemetry.addData("Horizontal Slide Position: ", slideHorizontalMotor.currentPosition)
            telemetry.addData("Tape Measure Motor: ", tapeMeasureRotateMotor.currentPosition)
            telemetry.addData("Rotate Servo Position: ", clawRotateServo.position)
            telemetry.addData("stuff: ", -((2 / (1 + (exp(-(slideVerticalMotor.targetPosition -slideVerticalMotor.currentPosition).toDouble() / MagicEquationSpeed)))) - 1) * (MagicEquationMax))
            telemetry.addLine("OpMode is active")
            telemetry.update()


        }
    }
};