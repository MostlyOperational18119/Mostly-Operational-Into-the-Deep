package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import kotlin.math.abs

@TeleOp(name = "Meet2Good\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83", group = "Aardvark")
class Meet2Good: Methods() {
    enum class VerticalSlideState { Floor, Low, High, Manual, Bar }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Pickup, Transfer }
    enum class AutomaticMovementState { Manual, Auto }
    enum class  HangStates { Up, Down, Reset, None}

    override fun runOpMode() {
        insideJokes ()

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
        setMotorModePositionNoReset(slideVerticalMotor)
        setMotorModePositionNoReset(hangerMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor)
        slideHorizontalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoderNoReset(tapeMeasureRotateMotor)
        setMotorModeEncoderNoReset(hangerMotor)
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
        val hangTouch = hardwareMap.touchSensor["HangTouch"]

        //VARIABLES
        val verticalSlideLow = 2300
        val verticalSlideHigh = 3650
        val horizontalSlideExtend = 1600
        val speedDiv = 2.3
        var singleRunCheck = 1
        var moveRotateServo = false
        var horizontalSlideToggle = HorizontalSlideState.Manual
        var verticalSlideToggle = VerticalSlideState.Manual
        var automatedTransferToggle = AutomaticTransferState.Pickup
        var automatedMovementToggle = AutomaticMovementState.Manual
        var hangerState = HangStates.None
        var intakeInToggle = false
        var intakeOutToggle = false
        val elapsedTime = ElapsedTime()

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
        elapsedTime.reset()

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
            //if (controller1.b&& !previousController1.b) {
            //    setMotorModePosition(slideVerticalMotor)
            //    setMotorModeEncoder(tapeMeasureRotateMotor)
            //    setMotorModePosition(slideHorizontalMotor)
            //    setMotorModeEncoder(hangerMotor)
            //}

            //HANGING
/*            if (controller2.a && !previousController2.a) {
                hangPusher.position = 0.00 //linear position here
                sleep(3000)
                hangerMotor.targetPosition = 100 //motor pos here
            }*/

          //if (controller1.left_bumper && !previousController1.left_bumper) {
          //    hangerState = HangStates.Up
          //}

           // if (controller1.right_bumper && !previousController1.right_bumper) {
            //    hangerState = HangStates.Reset
           // }

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
                    //if (controller1.right_trigger > 0.5){ clawServo.position = clawServoClosed }
                    //if (controller1.left_trigger  > 0.5){ clawServo.position = clawServoOpen }

                    //if (controller1.x && !previousController1.x){hangerMotor.targetPosition = 2000; hangerMotor.power = 0.2}
                    //if (controller1.y && !previousController1.y){hangerMotor.targetPosition = 0; hangerMotor.power = -0.2}
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
                            hangerMotor.power = 0.0
                        }
                        HangStates.Up -> {
                            hangerMotor.targetPosition = 12000
                            if (hangerMotor.targetPosition > hangerMotor.currentPosition) {
                                hangerMotor.power = 1.0
                            } else {
                                hangerMotor.power = -0.1
                            }
                        }
                        HangStates.Down -> {
                            hangerMotor.targetPosition = 0
                            if (hangerMotor.targetPosition > hangerMotor.currentPosition) {
                                hangerMotor.power = 0.1
                            } else {
                                hangerMotor.power = -1.0
                            }
                        }
                        HangStates.Reset -> {
                            while (!hangTouch.isPressed) {
                                hangerMotor.power = -0.5
                            }
                            hangerMotor.power = 0.0
                            setMotorModePosition(hangerMotor)
                            hangerState = HangStates.None
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
                        slideVerticalMotor.targetPosition = 0
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
                        slideVerticalMotor.targetPosition = 0
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
            telemetry.addLine(hangerMotor.targetPosition.toString())
            telemetry.addLine(hangTouch.isPressed.toString())
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