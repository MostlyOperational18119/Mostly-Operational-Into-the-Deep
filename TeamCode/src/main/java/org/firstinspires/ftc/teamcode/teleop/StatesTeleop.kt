package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.Methods

@TeleOp(name = "STATES TELEOP", group = "AAA")
class StatesTeleop: Methods() {
    override fun runOpMode() {
        initMotors()
        initServosAndSensorsSet()
        outRotationServo!!.position = outRotationCenter
        outSwivelServo!!.position = outSwivelPerpFront
        insideJokes()
        initOdometry()

        outClawToggle = false
        inClawToggle = false
        inRotationToggle = false
        outSwivelToggle = false
        transferServoToggle = false

        doOnce = false
        var barUp = true
        var transferSide = true
        var transferSideString = "Back Side"

        var outRotation = false
        var reverseThing = false
        var otherReverse = 1.0
        verticalHeight = 0
        speedDiv = 2.3
        var timeHor = 0.1

        waitForStart()

        while (opModeIsActive()) {
            telemetry.addData("Vertical Pos: ", slideVertical?.currentPosition)
            telemetry.addData("Horizontal Pos : ", slideHorizontal?.currentPosition)
            telemetry.addData("x: ", drive!!.poseEstimate.x)
            telemetry.addData("y: ", drive!!.poseEstimate.y)
            telemetry.addData("heading: ", drive!!.poseEstimate.heading)
            telemetry.addData("outRotationSerov: ", outRotationServo!!.position)
            telemetry.update()

            leftY1 = -gamepad1.left_stick_y.toDouble()/speedDiv * otherReverse
            leftX1 = gamepad1.left_stick_x.toDouble() * 2/(speedDiv) * otherReverse
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

                    if (controller1.right_trigger >0.5 && !(previousController1.right_trigger > 0.5)) { speedDiv = 1.0 }
                    if (!(controller1.left_trigger > 0.5) && !(controller1.right_trigger > 0.5))         { speedDiv = 2.3 }
                    if (controller1.left_bumper){
                        motorFL!!.power = -0.7
                        motorBL!!.power = 0.7
                        motorFR!!.power = 0.7
                        motorBR!!.power = -0.7
                    }
                    if (controller1.right_bumper){
                        motorFL!!.power = 0.7
                        motorBL!!.power = -0.7
                        motorFR!!.power = -0.7
                        motorBR!!.power = 0.7
                    }

                    if (controller1.a && !previousController1.a){ reverseThing = !reverseThing }
                    if (!reverseThing ){ otherReverse = 1.0}
                    if (reverseThing ){ otherReverse = -1.0}

                    if (controller1.dpad_up && !(previousController1.dpad_up)) {
                        drive!!.poseEstimate = Pose2d(47.44, -60.20, Math.toRadians(90.0))
                    }

                    if (controller1.dpad_down && !(previousController1.dpad_down)) {
                        val teleopBasket = drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(basketPose)
                            .setReversed(false)
                            .build()
                        drive!!.followTrajectorySequenceAsync(teleopBasket)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                    if (controller1.dpad_left && !previousController1.dpad_left) {
                        outSwivelServo!!.position = outSwivelPerpFront
                        outClawServo!!.position = outClawClose
                        outClawToggle = true
                        outRotationServo!!.position = outRotationUp
                        verticalSlideTo(verticalSlideBar, 1.0)
                        verticalHeight = verticalSlideBar

                        val teleopBar =  drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
                            .lineToLinearHeading(barPose)
                            .build()
                        drive!!.followTrajectorySequenceAsync(teleopBar)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                    if (controller1.dpad_right && !previousController1.dpad_right) {
                        outSwivelServo!!.position = outSwivelPerpBack
                        outClawServo!!.position = outClawOpen
                        outClawToggle = false
                        outRotationServo!!.position = outRotationBackWall
                        verticalSlideTo(0, 1.0)
                        verticalHeight = 0
                        val teleopPickup = drive!!.trajectorySequenceBuilder(drive!!.poseEstimate)
                            .setReversed(true)
                            .splineToConstantHeading(clipPickVector, Math.toRadians(-90.00))
                            .setReversed(false)
                            .build()
                        drive!!.followTrajectorySequenceAsync(teleopPickup)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                }
                AutomaticMovementState.Auto ->{
                    if (controller1.dpad_left && !previousController1.dpad_left) { drive!!.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (controller1.dpad_right && !previousController1.dpad_right) { drive!!.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (controller1.dpad_down && !previousController1.dpad_down) { drive!!.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (!drive!!.isBusy) { automatedMovementToggle = AutomaticMovementState.Manual }
                }
            }

            when (automaticTransferToggle) {
                AutomaticTransferState.Manual -> {
                    //AUTO TRANSFER
                    if (controller2.left_bumper && !(previousController2.left_bumper)) {
                        automaticTransferToggle = AutomaticTransferState.StartTransfer
                    }
                    if (controller1.b && !previousController1.b){
                        transferSide = !transferSide
                    }
                    if (transferSide){ transferSideString = "Back Side" }
                    else {transferSideString = "Front Side"}
                    telemetry.addData("Transfer Side: ", transferSideString)

                    //COLOR DETECT
                    colors = colorSensor!!.normalizedColors
                    if (colors!!.red > 0.6){ colorSeen = "red"}
                    else if (colors!!.blue > 0.6){ colorSeen = "blue"}
                    else if (colors!!.green > 0.2){ colorSeen = "yellow"}
                    else{ colorSeen = "none "}
                    telemetry.addData("Color Seen: ", colorSeen)

                    //OUT CLAW
                    if (controller2.right_bumper && !previousController2.right_bumper) { outClawToggle = !outClawToggle }
                    if (controller1.left_trigger >0.5 && !(previousController1.left_trigger > 0.5))   { outClawToggle = !outClawToggle}
                    if (!outClawToggle) { outClawServo!!.position = outClawOpen}
                    if (outClawToggle){ outClawServo!!.position = outClawClose}

                    //IN ROTATION
                    if (controller2.y && !previousController2.y) { inRotationToggle = !inRotationToggle}
                    if (!inRotationToggle) { inRotationServo!!.position = inRotationTransfer }
                    if (inRotationToggle)  { inRotationServo!!.position = inRotationPick }

                    //GATE SERVO
                    if (controller2.x && !previousController2.x){transferServoToggle = !transferServoToggle}
                    if (!transferServoToggle){transferServo!!.position = transferServoClose}
                    if (transferServoToggle) {transferServo!!.position = transferServoOpen }

                    //INTAKE MOTOR
                    if (controller2.right_trigger > 0.5 && !(previousController2.right_trigger > 0.5)){ intakeInToggle  = !intakeInToggle;  intakeOutToggle = false }
                    if (controller2.left_trigger > 0.5 && !(previousController2.left_trigger > 0.5)){ intakeOutToggle = !intakeOutToggle; intakeInToggle = false  }
                    if (intakeInToggle) { intakeMotor?.power = 0.5 }
                    else if (intakeOutToggle) { intakeMotor?.power = -0.8 }
                    else { intakeMotor?.power = 0.0 }

                    //IN STOP SERVO
                    if (slideHorizontal!!.currentPosition < 20){ inStopServo!!.position = inStopAutoOpen }
                    else { inStopServo!!.position = inStopClose }

                    //SWIVEL SERVO
//                    if (controller1.y && !previousController1.y) { outSwivelToggle = !outSwivelToggle }
//                    if (outSwivelToggle){ outSwivelServo!!.position = outSwivelParallel}
//                    if (!outSwivelToggle){ outSwivelServo!!.position = outSwivelPerpFront

                    //PLACING
                    if (controller1.y && !previousController1.y) {
                        verticalSlideTo(verticalSlideBar, 1.0)
                        verticalHeight = verticalSlideBar
                        outSwivelServo!!.position = outSwivelPerpFront
                        outRotationServo!!.position = outRotationFrontPlace
                        sleep(500)
                        outClawServo!!.position = outClawOpen
                        outClawToggle = false
                    }
                    if (controller1.x && !previousController1.x){
                        verticalSlideTo(verticalSlideBar, 1.0)
                        verticalHeight = verticalSlideBar
                        outSwivelServo!!.position = outSwivelPerpFront
                        outRotationServo!!.position = outRotationUp
                    }

                    //OUT ROTATION SERVO
                    if (controller2.b && !previousController2.b){
                        if (slideVertical!!.currentPosition > 800){
                            if (outRotation) {
                                outRotationServo!!.position = outRotationFrontOut
                                outSwivelServo!!.position = outSwivelPerpFront
                                outRotation = false
                            } else {
                                outRotationServo!!.position = outRotationBackOut
                                outSwivelServo!!.position = outSwivelPerpBack
                                outRotation = true
                            }
                        }
                        else{
                            outRotationServo!!.position = outRotationBackWall
                            outSwivelServo!!.position = outSwivelPerpBack
                        }
                    }

                    //HOR SLIDE
                    if (controller2.left_stick_button && !previousController2.left_stick_button) {
                        inRotationToggle = false
                        intakeInToggle = true
                        inStopServo!!.position = inStopAutoOpen
                        transferServoToggle = true
                        horizontalSlideToggle = HorizontalSlideState.Floor
                    }
                    when (horizontalSlideToggle) {
                        HorizontalSlideState.Manual -> {
                            if (leftY2!! > 0 && slideHorizontal!!.currentPosition < 950) {
                                horizontalSlideTo(950,(leftY2 as Double)*0.8)
                            } else if (leftY2!! < 0 && slideHorizontal!!.currentPosition > 0) {
                                horizontalSlideTo(0, -(leftY2 as Double)*0.8)
                            } else {
                                slideHorizontal!!.targetPosition = slideHorizontal!!.currentPosition
                                slideHorizontal!!.power = 0.1
                            }
                        }
                        HorizontalSlideState.Floor  -> { horizontalSlideTo(0,1.0);   horizontalBackToManual() }
                        HorizontalSlideState.Extend -> { horizontalSlideTo(950,1.0); horizontalBackToManual() }
                    }

                    // VERTICAL SLIDE
                    if (controller2.dpad_up && !previousController2.dpad_up) { verticalSlideToggle = VerticalSlideState.High; outRotationServo!!.position = outRotationBackOut}
                    if (controller2.dpad_right && !previousController2.dpad_right) { verticalSlideToggle = VerticalSlideState.Low; outRotationServo!!.position = outRotationBackOut}
                    if (controller2.dpad_down && !previousController2.dpad_down) { verticalSlideToggle = VerticalSlideState.Floor }
                    if (controller2.right_stick_button && !previousController2.right_stick_button) { verticalSlideToggle = VerticalSlideState.Floor; outRotationServo!!.position = outRotationCenter }
                    if (controller2.dpad_left && !previousController2.dpad_left) { verticalSlideToggle = VerticalSlideState.Bar; outRotationServo!!.position = outRotationUp; outSwivelServo!!.position = outSwivelPerpFront}
                    if (rightY2!! >= 0.2 || rightY2!! <= -0.2) { verticalSlideToggle = VerticalSlideState.Manual }
                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2!! > 0) {
                                verticalSlideTo(3000, (rightY2 as Double) / 1.3)
                            } else if (rightY2!! < 0 && slideVertical!!.currentPosition > 0) {
                                verticalSlideTo(0, -(rightY2 as Double) / 1.3)
                            }
                            // This part here is for holding power
                            else if (controller2.right_stick_y.toDouble() == 0.0 && previousController2.right_stick_y.toDouble() != 0.0) {
                                verticalHeight = slideVertical!!.currentPosition
                                verticalSlideTo(verticalHeight, 1.0)
                            } else {
                                verticalSlideTo(verticalHeight, 1.0)
                            }
                        }
                        VerticalSlideState.Floor -> { verticalSlideTo(0,0.8);    verticalBackToManual() }
                        VerticalSlideState.Low   -> { verticalSlideTo(verticalSlideLow,1.0); verticalBackToManual() }
                        VerticalSlideState.High  -> { verticalSlideTo(verticalSlideHigh,1.0); verticalBackToManual() }
                        VerticalSlideState.Bar  -> { verticalSlideTo(verticalSlideBar,1.0); verticalBackToManual() }
                    }
                }

                //TRANSFER SYSTEM
                AutomaticTransferState.StartTransfer-> {
                    outClawServo!!.position = outClawOpen
                    inRotationServo!!.position = inRotationTransferMinus
                    intakeMotor?.power = 1.0
                    horizontalSlideTo(-20,0.8)
                    verticalSlideTo(1300,1.0)
                    transferServo!!.position = transferServoClose
                    outSwivelServo!!.position = outSwivelPerpBack
                    if (!doOnce) {
                        elapsedTime.reset()
                        doOnce = true
                        timeHor = slideHorizontal!!.currentPosition * 0.0005
                    }
                    if (slideHorizontal!!.currentPosition < 10){
                        inRotationServo!!.position = inRotationTransfer
                        inStopServo!!.position = inStopAutoOpen
                    }
                    if (elapsedTime.time() > 0.15){
                        outRotationServo!!.position = outRotationCenter
                    }
                    if (elapsedTime.time() > 0.4){
                        verticalSlideTo(40,1.0)
                    }
                    if ((slideVertical!!.currentPosition < 70) && (elapsedTime.time() > timeHor + 0.7)){
                        automaticTransferToggle = AutomaticTransferState.Pickup
                        doOnce = false
                    }
                }
                AutomaticTransferState.Pickup -> {
                    outClawServo!!.position = outClawClose
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 0.4) {
                        automaticTransferToggle = AutomaticTransferState.ResetSlide
                        doOnce = false
                    }
                }
                AutomaticTransferState.ResetSlide ->{
                    intakeMotor!!.power = 0.0
                    verticalSlideTo(verticalSlideHigh,1.0)
                    verticalHeight = verticalSlideHigh
                    if (slideVertical!!.currentPosition > 200) {
                        automaticTransferToggle = AutomaticTransferState.RotateOut
                        doOnce = false
                    }
                }
                AutomaticTransferState.RotateOut ->{
                    if (transferSide) {
                        outRotationServo!!.position = outRotationBackOut
                        outSwivelServo!!.position = outSwivelPerpBack
                    }
                    else{
                        outRotationServo!!.position = outRotationFrontOut
                        outSwivelServo!!.position = outSwivelPerpFront
                    }
                    inRotationToggle = false
                    intakeInToggle = false
                    outClawToggle = true
                    automaticTransferToggle = AutomaticTransferState.Manual
                }
            }
        }
    }
}