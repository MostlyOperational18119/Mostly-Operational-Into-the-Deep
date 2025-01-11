package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import kotlin.math.abs

@TeleOp(name = "Meet 3 Teleop", group = "AAA")
class Meet3Teleop: Methods() {
    override fun runOpMode() {
        initMotors()
        initOdometry()
        initServosAndTouchWithoutSet()
        insideJokes()

        drive!!.poseEstimate = PoseStorage.currentPose

        outClawToggle = false
        inClawToggle = false
        inRotationToggle = false
        outSwivelToggle = false
        transferServoToggle = false
        doOnce = false
        var outRotation = false
        verticalHeight = 0
        speedDiv = 2.3

        waitForStart()

        while (opModeIsActive()) {
            leftY1 = -gamepad1.left_stick_y.toDouble()/speedDiv
            leftX1 = gamepad1.left_stick_x.toDouble()/(speedDiv/1.5)
            rightX1 = gamepad1.right_stick_x.toDouble()/speedDiv
            leftY2 = -gamepad2.left_stick_y.toDouble()
            rightY2 = -gamepad2.right_stick_y.toDouble()

            previousController1.copy(controller1)
            previousController2.copy(controller2)
            controller1.copy(gamepad1)
            controller2.copy(gamepad2)

            drive!!.update()
            drive!!.updatePoseEstimate()
            when (automatedMovementToggle) {
                AutomaticMovementState.Manual -> {
                    motorFL!!.power = (leftY1!! + leftX1!! + rightX1!!)
                    motorBL!!.power = (leftY1!! - leftX1!! + rightX1!!)
                    motorFR!!.power = (leftY1!! - leftX1!! - rightX1!!)
                    motorBR!!.power = (leftY1!! + leftX1!! - rightX1!!)

                    /*if (controller1.dpad_right && !(previousController1.dpad_right)) {
                        val teleopBasket = drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(basketPose)
                            .setReversed(false)
                            .build()
                        drive!!.followTrajectorySequenceAsync(teleopBasket)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                    if (controller1.dpad_left && !previousController1.dpad_left) {
                        val teleopBar =  drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(barPose)
                            .setReversed(false)
                            .build()
                        drive!!.followTrajectorySequenceAsync(teleopBar)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }*/
                }
                AutomaticMovementState.Auto ->{
                    if (controller1.dpad_left && !previousController1.dpad_left) { drive!!.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (controller1.dpad_right && !previousController1.dpad_right) { drive!!.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (!drive!!.isBusy) { automatedMovementToggle = AutomaticMovementState.Manual }
                }
            }

            when (automaticTransferToggle) {
                AutomaticTransferState.Manual -> {
                    // Auto transfer toggle
                    if (controller2.x && !(previousController2.x)) {
                        automaticTransferToggle = AutomaticTransferState.StartTransfer
                    }

                    //SERVOS
                    if (controller2.right_bumper && !previousController2.right_bumper) { outClawToggle = !outClawToggle }
                    if (outClawToggle){ outClawServo!!.position = outClawOpen}
                    if (!outClawToggle){ outClawServo!!.position = outClawClose}

                    if (controller2.left_bumper && !previousController2.left_bumper) { inClawToggle = !inClawToggle }
                    if (inClawToggle){ inClawServo!!.position = inClawOpen}
                    if (!inClawToggle){ inClawServo!!.position = inClawClose}

                    if (controller2.y && !previousController2.y) { inRotationToggle = true }
                    if (controller2.a && !previousController2.a) { inRotationToggle = false }
                    if (inRotationToggle){ inRotationServo!!.position = inRotationTransfer}
                    if (!inRotationToggle){ inRotationServo!!.position = inRotationPick}
//
//                    if (controller1.y && !previousController1.y) { outSwivelToggle = !outSwivelToggle }
//                    if (outSwivelToggle){ outSwivelServo!!.position = outSwivelParallel}
//                    if (!outSwivelToggle){ outSwivelServo!!.position = outSwivelPerpFront}
                    if (controller1.left_bumper && !previousController1.left_bumper)   { speedDiv = 4.6 }
                    if (controller1.right_bumper && !previousController1.right_bumper) { speedDiv = 1.0 }
                    if (!controller1.left_bumper && !controller1.right_bumper)         { speedDiv = 2.3 }

                    if (controller2.b && !previousController2.b){
                        if (slideVertical!!.currentPosition < 1500) {
                            verticalSlideTo(1000,1.0)
                            verticalHeight = 1000
                        }
                        sleep(300)
                        if (outRotation) {
                            outRotationServo!!.position = outRotationFront
                            outSwivelServo!!.position = outSwivelPerpFront
                            outRotation = false
                        } else {
                            outRotationServo!!.position = outRotationBack
                            outSwivelServo!!.position = outSwivelPerpBack
                            outRotation = true
                        }
                    }

                    if (controller2.left_trigger >= 0.5 && previousController2.left_trigger < 0.5 && IntakeRotation > 0)
                    {
                        IntakeRotation -= 1
                    }
                    if (controller2.right_trigger >= 0.5 && previousController2.right_trigger < 0.5 && IntakeRotation < 5)
                    {
                        IntakeRotation += 1
                    }

                    when (IntakeRotation) {
                        1 -> {inSwivelServo!!.position = inSwivelLeft90}
                        2 -> {inSwivelServo!!.position = inSwivelLeft}
                        3 -> {inSwivelServo!!.position = inSwivelCenter}
                        4 -> {inSwivelServo!!.position = inSwivelRight}
                        5 -> {inSwivelServo!!.position = inSwivelRight90}
                    }

                    //HOR SLIDE
                    if (controller2.left_stick_button && !previousController2.left_stick_button) { horizontalSlideToggle = HorizontalSlideState.Floor }
                    //if (controller2.x && !previousController2.x) { horizontalSlideToggle = HorizontalSlideState.Extend }
                    if (leftY2!! >= 0.2 || leftY2!! <= -0.2) { horizontalSlideToggle = HorizontalSlideState.Manual }
                    when (horizontalSlideToggle) {
                        HorizontalSlideState.Manual -> {
                            if (leftY2!! > 0 && slideHorizontal!!.currentPosition < 950) {
                                horizontalSlideTo(950,leftY2 as Double)
                            } else if (leftY2!! < 0 && slideHorizontal!!.currentPosition > 0) {
                                horizontalSlideTo(0, -(leftY2 as Double))
                            } else {
                                slideHorizontal!!.targetPosition = slideHorizontal!!.currentPosition
                                slideHorizontal!!.power = 0.1
                            }
                        }
                        HorizontalSlideState.Floor  -> { horizontalSlideTo(0,1.0);   horizontalBackToManual() }
                        HorizontalSlideState.Extend -> { horizontalSlideTo(950,1.0); horizontalBackToManual() }
                    }

                    // Vertical slide
                    if (controller2.dpad_up && !previousController2.dpad_up) { verticalSlideToggle = VerticalSlideState.High }
                    if (controller2.dpad_down && !previousController2.dpad_down) { verticalSlideToggle = VerticalSlideState.Floor }
                    if (controller2.right_stick_button && !previousController2.right_stick_button) { verticalSlideToggle = VerticalSlideState.Floor }
                    if (controller2.dpad_left && !previousController2.dpad_left) { verticalSlideToggle = VerticalSlideState.Low }
                    if (controller2.dpad_right && !previousController2.dpad_right) { verticalSlideToggle = VerticalSlideState.Bar }
                    if (rightY2!! >= 0.2 || rightY2!! <= -0.2) { verticalSlideToggle = VerticalSlideState.Manual }
                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2!! > 0 && slideVertical!!.currentPosition < 3700) {
                                verticalSlideTo(3700, (rightY2 as Double) / 3)
                            } else if (rightY2!! < 0 && slideVertical!!.currentPosition > 0) {
                                verticalSlideTo(0, -(rightY2 as Double) / 3)
                            }
                            // This part here is for holding power
                            else if (controller2.right_stick_y.toDouble() == 0.0 && previousController2.right_stick_y.toDouble() != 0.0) {
                                verticalHeight = slideVertical!!.currentPosition
                                verticalSlideTo(verticalHeight, 0.3)
                            } else {
                                verticalSlideTo(verticalHeight, 0.3)
                            }
                        }
                        VerticalSlideState.Floor -> { verticalSlideTo(0,1.0);    verticalBackToManual() }
                        VerticalSlideState.Low   -> { verticalSlideTo(1000,1.0); verticalBackToManual() }
                        VerticalSlideState.High  -> { verticalSlideTo(3500,1.0); verticalBackToManual() }
                        VerticalSlideState.Bar   -> { verticalSlideTo(1500,1.0); verticalBackToManual() }
                    }
                }

                // Automated transfer
                AutomaticTransferState.StartTransfer-> {
                    inClawServo!!.position = inClawClose
                    inSwivelServo!!.position = inSwivelCenter
                    inRotationServo!!.position = inRotationTransfer
                    horizontalSlideTo(0,1.0)
                    verticalSlideTo(1200,1.0)
                    outRotationServo!!.position = outRotationCenter
                    outSwivelServo!!.position = outSwivelParallel
                    outClawServo!!.position = outClawOpen
                    transferServo!!.position = transferServoIntake
                    // Once the vertical slide is up, then we use a timer to wait and then change state
                    if (abs(1200 - slideVertical!!.currentPosition)  < 20){
                        // The doOnce thingy is just to make it so that it only resets timer once.
                        if (!doOnce) { elapsedTime.reset(); doOnce = true }
                        if (elapsedTime.time() > 0.8){
                            inClawServo!!.position = inClawOpen
                        }
                        if (elapsedTime.time() > 1.0){
                            automaticTransferToggle = AutomaticTransferState.InRotate
                            doOnce = false
                        }
                    }
                }
                AutomaticTransferState.InRotate ->{
                    inRotationServo!!.position = inRotationPick
                    if (!doOnce) { elapsedTime.reset();doOnce = true }
                    if (elapsedTime.time() > 0.5) {
                        automaticTransferToggle = AutomaticTransferState.SlideDown
                        doOnce = false
                    }
                }
                AutomaticTransferState.SlideDown ->{
                    transferServo!!.position = transferServoNormal
                    verticalSlideTo(0,1.0)
                    if (!doOnce) { elapsedTime.reset();doOnce = true }
                    if (elapsedTime.time() > 0.5) {
                        automaticTransferToggle = AutomaticTransferState.Pickup
                        doOnce = false
                    }
                }
                AutomaticTransferState.Pickup -> {
                    outClawServo!!.position = outClawClose
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 0.5) {
                        automaticTransferToggle = AutomaticTransferState.ResetSlide
                        doOnce = false
                    }
                }
                AutomaticTransferState.ResetSlide ->{
                    verticalSlideTo(1500,1.0)
                    verticalHeight = 1500
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 1.0) {
                        automaticTransferToggle = AutomaticTransferState.RotateOut
                        doOnce = false
                    }
                }
                AutomaticTransferState.RotateOut ->{
                    inRotationServo!!.position = inRotationPick
                    transferServo!!.position = transferServoNormal
                    outRotationServo!!.position = outRotationBack
                    outSwivelServo!!.position = outSwivelPerpBack
                    automaticTransferToggle = AutomaticTransferState.Manual
                }
            }
        }
    }
}