package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs

@TeleOp(name = "Meet2Good\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83", group = "Aardvark")
class Meet2Good: DriveMethods() {
    enum class VerticalSlideState { Floor, Low, High, Manual, Bar }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Pickup, Transfer }
    enum class AutomaticMovementState { Manual, Auto }
    enum class  HangStates { Up, Down, Reset, None}

    override fun runOpMode() {
        telemetry.addLine(when ((0..50).random()) {
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
            49 -> "https://lhohq.info"
            else -> "Why did we add these?"
        })
        telemetry.update()

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
        setMotorModePosition(hangerMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(tapeMeasureRotateMotor)
        setMotorModeEncoder(hangerMotor)
        tapeMeasureRotateMotor.targetPosition = 0
//        setMotorModeEncoder(grabberExtensionMotor)
//        setMotorModeEncoder(tapeMeasureRotateMotor)

        //SERVOS
        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawServo = hardwareMap.servo["clawServo"]
        val transferServo = hardwareMap.servo["transferServo"]
        val clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo.position = clawRotateUpRight
        val hangPusher = hardwareMap.servo["hangPusher"] // Linear servo

        //VARIABLES
        val verticalSlideLow = 2300
        val verticalSlideHigh = 3650
        val horizontalSlideExtend = 1600
        val speedDiv = 2
        var singleRunCheck = 1
        var moveRotateServo = false
        var horizontalSlideToggle = HorizontalSlideState.Manual
        var verticalSlideToggle = VerticalSlideState.Manual
        var automatedTransferToggle = AutomaticTransferState.Pickup
        var automatedMovementToggle = AutomaticMovementState.Manual
        var hangerState = HangStates.Reset
        var intakeInToggle = false
        var intakeOutToggle = false

        //ROADRUNNER
        val drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = PoseStorage.currentPose
        val basketVector = Vector2d(-58.26, -57.64)
        val basketHeading = Math.toRadians(225.00)
        //val basketPose = Pose2d(-58.26, -57.64, 225.0)
        val barVector = Vector2d(-10.04, -34.01)
        val barHeading = Math.toRadians(-90.00)
        //val barPose = Pose2d(-10.04, -34.01, -90.0)

        waitForStart()

        while (opModeIsActive()) {
            //GAME PADS
            previousController1.copy(controller1)
            previousController2.copy(controller2)
            controller1.copy(gamepad1)
            controller2.copy(gamepad2)

            //INPUTS
            val leftY = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val leftX = gamepad1.left_stick_x.toDouble()
            val rightX = gamepad1.right_stick_x.toDouble()
            val leftY2 = -gamepad2.left_stick_y.toDouble()
            val rightY2 = -gamepad2.right_stick_y.toDouble()

            //MOVEMENT + ODOMETRY
            drive.update()
            when (automatedMovementToggle) {
                AutomaticMovementState.Manual ->{
                    //MOVE
                    motorFL.power = (leftY + leftX + rightX) / speedDiv
                    motorBL.power = (leftY - leftX + rightX) / speedDiv
                    motorFR.power = (leftY - leftX - rightX) / speedDiv
                    motorBR.power = (leftY + leftX - rightX) / speedDiv

//                    if (controller1.left_trigger>0.5) {
//                        val traj1 =  drive.trajectorySequenceBuilder(drive.poseEstimate)
//                            .setReversed(false)
//                            .splineTo(basketVector, basketHeading)
//                            .build()
//                        drive.followTrajectorySequenceAsync(traj1)
//                        automatedMovementToggle = AutomaticMovementState.Auto
//                    }
//                    if (controller1.left_bumper) {
//                        val traj1 =  drive.trajectorySequenceBuilder(drive.poseEstimate)
//                            .setReversed(false)
//                            .splineTo(barVector, barHeading)
//                            .build()
//                        drive.followTrajectorySequenceAsync(traj1)
//                        automatedMovementToggle = AutomaticMovementState.Auto
//                    }
                }

                AutomaticMovementState.Auto ->{
                    if (!drive.isBusy) { automatedMovementToggle = AutomaticMovementState.Manual }
                }
            }

            //RESET
            if (controller1.b&& !previousController1.b) {
                setMotorModePosition(slideVerticalMotor)
                setMotorModeEncoder(tapeMeasureRotateMotor)
                setMotorModePosition(slideHorizontalMotor)
                setMotorModeEncoder(hangerMotor)
            }

            //HANGING
//            if (controller2.a && !previousController2.a) {
//                hangPusher.position = 0.00 //linear position here
//                sleep(3000)
//                hangerMotor.targetPosition = 100 //motor pos here
//            }

//          if (gamepad1.left_trigger >= 0.2) {
//              tapeMeasureRotateMotor.power = 1.0
//          } else if (gamepad1.right_trigger >= 0.2) {
//              tapeMeasureRotateMotor.power = -1.0
//          } else {
//              tapeMeasureRotateMotor.power = 0.0
//          }

            when (automatedTransferToggle) {
                AutomaticTransferState.Pickup -> {
                    //TRANSFER TOGGLE
                    //if (controller2.b && !previousController2.b){ automatedTransferToggle = AutomaticTransferState.Transfer }

                    //INTAKE SERVO
                    if (controller2.x&& !previousController2.x){
                        intakeInToggle = !intakeInToggle
                        intakeOutToggle = false
                    }
                    if (controller2.b&& !previousController2.b){
                        intakeOutToggle = !intakeOutToggle
                        intakeInToggle = false
                    }
                    if (intakeInToggle) { intakeServo?.power = 1.0 }
                    else if (intakeOutToggle) { intakeServo?.power = -1.0 }
                    else { intakeServo?.power = 0.0 }

                    if (controller2.y && !previousController2.y && clawServo.position == clawServoOpen){
                        clawServo.position = clawServoClosed
                    }
                    else if (controller2.y && !previousController2.y) {
                        clawServo.position = clawServoOpen
                    }

                    if (controller2.a && !previousController2.a){ automatedTransferToggle = AutomaticTransferState.Transfer }

                    if (leftY2 >= 0.2 || leftY2 <= -0.2){verticalSlideToggle = VerticalSlideState.Manual}
                    if (leftY >= 0.2 || leftY <= -0.2){horizontalSlideToggle = HorizontalSlideState.Manual}

                    if (controller2.left_stick_button && !previousController2.left_stick_button) {
                        transferServo.position = transferUpPos
                        if (slideHorizontalMotor.currentPosition <= 400) {
                            horizontalSlideToggle = HorizontalSlideState.Extend
                        } else {
                            horizontalSlideToggle = HorizontalSlideState.Floor
                        }
                    }

                    //CLAW SERVO1
                    if (controller1.right_trigger > 0.5){ clawServo.position = clawServoClosed }
                    if (controller1.left_trigger  > 0.5){ clawServo.position = clawServoOpen }

                    if (controller1.x && !previousController1.x){hangerMotor.targetPosition = 2000; hangerMotor.power = 0.2}
                    if (controller1.y && !previousController1.y){hangerMotor.targetPosition = 0; hangerMotor.power = -0.2}
                    if (controller1.a && !previousController1.a){hangPusher.position = servoHangActive}
                    if (controller1.b && !previousController1.b){hangPusher.position = servoHangPassive}

                    if (controller2.dpad_left && !previousController2.dpad_left){clawRotateServo.position = clawRotateWall}
                    if (controller2.dpad_right && !previousController2.dpad_right){clawRotateServo.position = clawRotateUpRight}
                    if (controller2.dpad_down && !previousController2.dpad_down){ transferServo.position = transferDownPos }
                    if (controller2.dpad_up  && !previousController2.dpad_up ){ transferServo.position = transferUpPos }

                    //ROTATE SERVO
                    //if (controller2.right_trigger > 0.5){ clawRotateServo.position = clawRotateOut }
                    //if (controller2.left_trigger  > 0.5){ clawRotateServo.position = clawRotateUpRight }

                    //HORIZONTAL MOTOR
                    if (controller2.y && !previousController2.y) {horizontalSlideToggle = HorizontalSlideState.Floor }
                    //if (controller2.x && !previousController2.x) {horizontalSlideToggle = HorizontalSlideState.Extend }

                    when (horizontalSlideToggle) {
                        HorizontalSlideState.Manual -> {
                            if (leftY2 > 0.0 && slideHorizontalMotor.currentPosition < horizontalSlideExtend) {
                                slideHorizontalMotor.targetPosition = horizontalSlideExtend
                                slideHorizontalMotor.power = leftY2/1.5
                            } else if (leftY2 < 0.0 && slideHorizontalMotor.currentPosition > 0) {
                                slideHorizontalMotor.targetPosition = 0
                                slideHorizontalMotor.power = leftY2/1.5
                            }
                            else {
                                slideHorizontalMotor.targetPosition = slideHorizontalMotor.currentPosition
                                slideHorizontalMotor.power = 0.1
                            }

                        }
                        HorizontalSlideState.Floor -> {
                            slideHorizontalMotor.targetPosition = 0
                            slideHorizontalMotor.power = -0.8
                            if (abs(slideHorizontalMotor.currentPosition) < 50) {
                                horizontalSlideToggle = HorizontalSlideState.Manual
                            }
                        }
                        HorizontalSlideState.Extend -> {
                            slideHorizontalMotor.targetPosition = horizontalSlideExtend
                            slideHorizontalMotor.power = 0.8
                            if (abs(slideVerticalMotor.currentPosition - horizontalSlideExtend) < 50) {
                                horizontalSlideToggle = HorizontalSlideState.Manual
                            }
                        }
                    }

                    when (hangerState) {
                        HangStates.None -> {
                            hangerMotor.power = 0.0 //HangTouch
                        }
                        HangStates.Up -> {
                            slideVerticalMotor.targetPosition = 5800
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 1.0
                            } else {
                                slideVerticalMotor.power = -0.1
                            }
                        }
                        HangStates.Down -> {
                            slideVerticalMotor.targetPosition = 0
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.1
                            } else {
                                slideVerticalMotor.power = -1.0
                            }
                        }
                        HangStates.Reset -> {

                        }
                    }

                    //VERTICAL SLIDE
                    if (controller2.right_stick_button  && !previousController2.left_stick_button) { verticalSlideToggle = VerticalSlideState.Floor; clawRotateServo.position = clawRotateUpRight }
                    if (controller2.right_bumper  && !previousController2.right_bumper) { verticalSlideToggle = VerticalSlideState.Low; clawRotateServo.position = clawRotateStraight }
                    if (controller2.right_trigger >= 0.5)   { verticalSlideToggle = VerticalSlideState.High; clawRotateServo.position = clawRotateOut }
                    if (controller2.left_bumper && !previousController2.left_bumper){ verticalSlideToggle = VerticalSlideState.Bar; clawRotateServo.position = clawRotateStraight}

                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2 > 0.0 && slideVerticalMotor.currentPosition < verticalSlideHigh) {
                                slideVerticalMotor.targetPosition +=40
                            } else if (rightY2 < 0.0 && slideVerticalMotor.currentPosition > 0) {
                                slideVerticalMotor.targetPosition -=40
                            }
                            slideVerticalMotor.power = 1.0
                        }
                        VerticalSlideState.Floor -> {
                            slideVerticalMotor.targetPosition = 0
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition) < 20) {
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }
                        VerticalSlideState.Low -> {
                            slideVerticalMotor.targetPosition = verticalSlideLow
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - verticalSlideLow) < 20) {
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }
                        VerticalSlideState.High -> {
                            slideVerticalMotor.targetPosition = verticalSlideHigh
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - verticalSlideHigh) < 20) {
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }
                        VerticalSlideState.Bar -> {
                            slideVerticalMotor.targetPosition = 1738
                            if (slideVerticalMotor.targetPosition > slideVerticalMotor.currentPosition) {
                                slideVerticalMotor.power = 0.8
                            } else {
                                slideVerticalMotor.power = -0.8
                            }
                            if (abs(slideVerticalMotor.currentPosition - verticalSlideHigh) < 20) {
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }
                    }
                }
                AutomaticTransferState.Transfer ->{
                    if (singleRunCheck == 1) {
                        intakeServo?.power = 0.0
                        slideHorizontalMotor.targetPosition = 0
                        slideHorizontalMotor.power = -0.8
                        slideVerticalMotor.targetPosition = 400
                        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
                        else { slideVerticalMotor.power = -0.8 }
                        transferServo.position = transferUpPos
                        clawServo.position = clawServoOpen
                        singleRunCheck=2
                    }

                    if (slideVerticalMotor.currentPosition < 600){
                        clawRotateServo.position = clawRotateRest
                        moveRotateServo = true
                    }

                    if (moveRotateServo){
                        sleep(500)
                        slideVerticalMotor.targetPosition = 80
                        if (slideVerticalMotor.currentPosition > slideVerticalMotor.targetPosition) { slideVerticalMotor.power = 0.8 }
                        else { slideVerticalMotor.power = -0.8 }
                        sleep(500)
                        clawServo.position = clawServoClosed
                        sleep(500)
                        slideVerticalMotor.targetPosition = 1000
                        slideVerticalMotor.power = 0.8
                        sleep(100)
                        clawRotateServo.position = clawRotateOut
                        sleep(200)
                        moveRotateServo = false
                        singleRunCheck = 1
                        intakeInToggle = false
                        intakeOutToggle = false
                        verticalSlideToggle = VerticalSlideState.Manual
                        horizontalSlideToggle = HorizontalSlideState.Manual
                        automatedTransferToggle = AutomaticTransferState.Pickup
                    }
                }
            }

            telemetry.addData("Odometry: ",
                String.format(
                    "Pose: %s, Velocity: %s",
                    drive.poseEstimate.toString(),
                    drive.getWheelVelocities().toString()
                )
            )
            telemetry.addLine(hangerMotor.currentPosition.toString())
            telemetry.addData("Vertical Slide Power: ", slideVerticalMotor.power)
            telemetry.addData("Vertical Slide Target: ", slideVerticalMotor.targetPosition)
            telemetry.addData("Vertical Slide Position: ", slideVerticalMotor.currentPosition)
            telemetry.addData("Horizontal Slide Power: ", slideHorizontalMotor.power)
            telemetry.addData("Horizontal Slide Target: ", slideHorizontalMotor.targetPosition)
            telemetry.addData("Horizontal Slide Position: ", slideHorizontalMotor.currentPosition)
            telemetry.addData("Tape Measure Motor: ", tapeMeasureRotateMotor.currentPosition)
            telemetry.addData("Rotate Servo Position: ", clawRotateServo.position)
            telemetry.addData("Transfer Servo Position: ", transferServo.position)
            telemetry.addLine("OpMode is active")

            telemetry.update()
        }
    }
}