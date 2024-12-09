package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive

@TeleOp(name = "Meet2Good\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83\uD83E\uDD83", group = "AAAA")
class Meet2Good: Methods() {
    override fun runOpMode() {
        insideJokes ()
        initMotorsNoReset()
        initServosAndTouch()

        val drive = SampleMecanumDrive(hardwareMap)
        drive.poseEstimate = PoseStorage.currentPose

        waitForStart()

        while (opModeIsActive()) {
            copyControls ()

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
                    motorFL!!.power = (leftY + leftX + rightX) / speedDiv
                    motorBL!!.power = (leftY - leftX + rightX) / speedDiv
                    motorFR!!.power = (leftY - leftX - rightX) / speedDiv
                    motorBR!!.power = (leftY + leftX - rightX) / speedDiv

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

            when (automatedTransferToggle) {
                AutomaticTransferState.Pickup -> {
                    //TRANSFER TOGGLE
                    if (controller2.a && !previousController2.a){ automatedTransferToggle = AutomaticTransferState.Transfer }

                    //INTAKE SERVO
                    if (controller2.x&& !previousController2.x){ intakeInToggle  = !intakeInToggle;  intakeOutToggle = false }
                    if (controller2.b&& !previousController2.b){ intakeOutToggle = !intakeOutToggle; intakeInToggle = false  }
                    if (intakeInToggle) { intakeServo?.power = 1.0 }
                    else if (intakeOutToggle) { intakeServo?.power = -1.0 }
                    else { intakeServo?.power = 0.0 }

                    //CLAW SERVO
                    if (controller2.y && !previousController2.y && clawServo!!.position == clawServoOpen){ clawServo!!.position = clawServoClosed }
                    else if (controller2.y && !previousController2.y) { clawServo!!.position = clawServoOpen }

                    //HANGING
                    if (controller1.a && !previousController1.a){hangPusher!!.position = servoHangActive}
                    if (controller1.b && !previousController1.b){hangPusher!!.position = servoHangPassive}

                    //ROTATE SERVO
                    if (controller2.dpad_left && !previousController2.dpad_left){clawRotateServo!!.position = clawRotateWall}
                    if (controller2.dpad_right && !previousController2.dpad_right){clawRotateServo!!.position = clawRotateUpRight}

                    //TRANSFER SERVO
                    if (controller2.dpad_down && !previousController2.dpad_down){ transferServo!!.position = transferDownPos }
                    if (controller2.dpad_up  && !previousController2.dpad_up ){ transferServo!!.position = transferUpPos }

                    //HORIZONTAL MOTOR
                    if (controller2.y && !previousController2.y) {horizontalSlideToggle = HorizontalSlideState.Floor }
                    if (leftY >= 0.2 || leftY <= -0.2){horizontalSlideToggle = HorizontalSlideState.Manual}
                    if (controller2.left_stick_button && !previousController2.left_stick_button) {
                        transferServo!!.position = transferUpPos
                        horizontalSlideToggle =
                            if (slideHorizontalMotor!!.currentPosition <= 400) { HorizontalSlideState.Extend }
                            else { HorizontalSlideState.Floor }
                    }

                    when (horizontalSlideToggle) {
                        HorizontalSlideState.Manual -> {
                            if (leftY2 > 0.0 && slideHorizontalMotor!!.currentPosition < horizontalSlideExtend) {
                                horizontalSlideTo(horizontalSlideExtend,leftY2/1.5)
                            } else if (leftY2 < 0.0 && slideHorizontalMotor!!.currentPosition > 0) {
                                horizontalSlideTo(0,leftY2/1.5)
                            } else { horizontalSlideTo(slideHorizontalMotor!!.currentPosition,0.1) }
                        }
                        HorizontalSlideState.Floor -> { horizontalSlideTo(0,1.0); horizontalBackToManual() }
                        HorizontalSlideState.Extend -> { horizontalSlideTo(horizontalSlideExtend,1.0); horizontalBackToManual() }
                    }

                    //VERTICAL SLIDE
                    if (controller2.right_stick_button  && !previousController2.left_stick_button) { verticalSlideToggle = VerticalSlideState.Floor; clawRotateServo!!.position = clawRotateUpRight }
                    if (controller2.right_bumper  && !previousController2.right_bumper) { verticalSlideToggle = VerticalSlideState.Low; clawRotateServo!!.position = clawRotateStraight }
                    if (controller2.right_trigger >= 0.5)   { verticalSlideToggle = VerticalSlideState.High; clawRotateServo!!.position = clawRotateOut }
                    if (controller2.left_bumper && !previousController2.left_bumper){ verticalSlideToggle = VerticalSlideState.Bar; clawRotateServo!!.position = clawRotateStraight}
                    if (leftY2 >= 0.2 || leftY2 <= -0.2){verticalSlideToggle = VerticalSlideState.Manual}

                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2 > 0.0 && slideVerticalMotor!!.currentPosition < verticalSlideHigh) { slideVerticalMotor!!.targetPosition +=40
                            } else if (rightY2 < 0.0 && slideVerticalMotor!!.currentPosition > 0) { slideVerticalMotor!!.targetPosition -=40 }
                            slideVerticalMotor!!.power = 1.0
                        }
                        VerticalSlideState.Floor -> { verticalSlideTo(0, 1.0);verticalBackToManual() }
                        VerticalSlideState.Low -> { verticalSlideTo(verticalSlideLow, 1.0);verticalBackToManual() }
                        VerticalSlideState.High -> { verticalSlideTo(verticalSlideHigh, 1.0);verticalBackToManual() }
                        VerticalSlideState.Bar -> { verticalSlideTo(verticalSlideBar, 1.0);verticalBackToManual() }
                    }

                    //HANGING
//                    if (controller2.a && !previousController2.a) {
//                        hangPusher!!.position = 0.00 //linear position here
//                        sleep(3000)
//                        hangerMotor!!.targetPosition = 100 //motor pos here
//                    }
//                    if (controller1.left_bumper && !previousController1.left_bumper) { hangerState = HangStates.Up }
//                    if (controller1.right_bumper && !previousController1.right_bumper) { hangerState = HangStates.Reset }

//                    when (hangerState) {
//                        HangStates.None -> { hangerMotor!!.power = 0.0 }
//                        HangStates.Up -> {
//                            hangerMotor!!.targetPosition = 12000
//                            if (hangerMotor!!.targetPosition > hangerMotor!!.currentPosition) {
//                                hangerMotor!!.power = 1.0
//                            } else {
//                                hangerMotor!!.power = -0.1
//                            }
//                        }
//                        HangStates.Down -> {
//                            hangerMotor!!.targetPosition = 0
//                            if (hangerMotor!!.targetPosition > hangerMotor!!.currentPosition) {
//                                hangerMotor!!.power = 0.1
//                            } else {
//                                hangerMotor!!.power = -1.0
//                            }
//                        }
//                        HangStates.Reset -> {
//                            while (!hangTouch!!.isPressed) {
//                                hangerMotor!!.power = -0.5
//                            }
//                            hangerMotor!!.power = 0.0
//                            setMotorModePosition(hangerMotor!!)
//                            hangerState = HangStates.None
//                        }
//                    }
                }

                AutomaticTransferState.Transfer ->{
                    if (singleRunCheck == 1) {
                        intakeServo?.power = 0.0
                        slideHorizontalMotor!!.targetPosition = 0
                        slideHorizontalMotor!!.power = -0.8
                        verticalSlideTo(0, 1.0)
                        transferServo!!.position = transferUpPos
                        clawServo!!.position = clawServoOpen
                        singleRunCheck=2
                    }
                    if (slideVerticalMotor!!.currentPosition < 600){
                        clawRotateServo!!.position = clawRotateRest
                        moveRotateServo = true
                    }
                    if (moveRotateServo){
                        sleep(500)
                        verticalSlideTo(0, 1.0)
                        sleep(500)
                        clawServo!!.position = clawServoClosed
                        sleep(500)
                        verticalSlideTo(1000, 1.0)
                        sleep(100)
                        clawRotateServo!!.position = clawRotateOut
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
            telemetry.addLine(hangerMotor!!.currentPosition.toString())
            telemetry.addLine(hangerMotor!!.targetPosition.toString())
            telemetry.addLine(hangTouch!!.isPressed.toString())
            telemetry.addData("Vertical Slide Power: ", slideVerticalMotor!!.power)
            telemetry.addData("Vertical Slide Target: ", slideVerticalMotor!!.targetPosition)
            telemetry.addData("Vertical Slide Position: ", slideVerticalMotor!!.currentPosition)
            telemetry.addData("Horizontal Slide Power: ", slideHorizontalMotor!!.power)
            telemetry.addData("Horizontal Slide Target: ", slideHorizontalMotor!!.targetPosition)
            telemetry.addData("Horizontal Slide Position: ", slideHorizontalMotor!!.currentPosition)
            telemetry.addData("Tape Measure Motor: ", tapeMeasureRotateMotor!!.currentPosition)
            telemetry.addData("Rotate Servo Position: ", clawRotateServo!!.position)
            telemetry.addData("Transfer Servo Position: ", transferServo!!.position)
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}