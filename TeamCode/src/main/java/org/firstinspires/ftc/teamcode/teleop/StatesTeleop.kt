package org.firstinspires.ftc.teamcode.teleop

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
//import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.Methods
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable

@TeleOp(name = "STATES TELEOP", group = "AAA")
class StatesTeleop: Methods() {
    override fun runOpMode() {
        initMotors()
        initServosAndTouchWithoutSet()
        outRotationServo!!.position = outRotationCenter
        insideJokes()

        val drive1 = SampleMecanumDriveCancelable(hardwareMap)
        drive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
        drive1.poseEstimate = PoseStorage.currentPose

        //val limelight3A = hardwareMap.get(Limelight3A::class.java, "limelight")

        //switchPipelineEnum(limelight3A, PipelineType.Orange)

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
        //colorSensor?.gain = 50.0F

        waitForStart()

        while (opModeIsActive()) {
            //val results = limelight3A.latestResult!!
            //val colorResults = results.colorResults

            telemetry.addData("Vertical Pos: ", slideVertical?.currentPosition)
            telemetry.addData("Horizontal Pos : ", slideHorizontal?.currentPosition)
            telemetry.addData("X: ", drive1.poseEstimate.x)
            telemetry.addData("Y: ", drive1.poseEstimate.y)
            telemetry.addData("heading: ", drive1.poseEstimate.heading)
            telemetry.addData("leftX1: ", leftX1)
            telemetry.update()

//            colorResults.forEach {
//                val resultPose = it.targetPoseCameraSpace
//                val resultPosition = resultPose.position
//                val resultOrientation = resultPose.orientation
//
//                telemetry.addLine("Detection Position: (${resultPosition.x}, ${resultPosition.y}, ${resultPosition.z})")
//                telemetry.addLine("Detection Orientation: (${resultOrientation.pitch}, ${resultOrientation.yaw}, ${resultOrientation.roll})")
//                telemetry.addLine()
//            }

            leftY1 = -gamepad1.left_stick_y.toDouble()/speedDiv * otherReverse
            leftX1 = gamepad1.left_stick_x.toDouble() * 1.5/(speedDiv) * otherReverse
            rightX1 = gamepad1.right_stick_x.toDouble()/speedDiv
            leftY2 = -gamepad2.left_stick_y.toDouble()
            rightY2 = -gamepad2.right_stick_y.toDouble()

            previousController1.copy(controller1)
            previousController2.copy(controller2)
            controller1.copy(gamepad1)
            controller2.copy(gamepad2)

            /*colors = colorSensor?.normalizedColors
            if (colors?.red!! <= 0.25 && colors?.green!! <= 0.25 && colors?.blue!! <= 0.25) {
                currentColor = ColorStates.None
            } else {
                currentHighestColorValue = colors?.red!!
                currentColor = ColorStates.Red
                if (colors!!.green >= currentHighestColorValue) {
                    currentHighestColorValue = colors?.green!!
                    currentColor = ColorStates.Yellow
                }
                if (colors!!.blue > currentHighestColorValue) {
                    currentHighestColorValue = colors?.blue!!
                    currentColor = ColorStates.Blue
                }
            }*/

            drive1.update()
            drive1.updatePoseEstimate()
            
//            if (controller1.cross && !previousController1.cross){
//                setMotorModeEncoder(intakeMotor!!)
//                setMotorModePosition(slideHorizontal!!)
//                setMotorModePosition(slideVertical!!)
//                motorFL!!.direction = DcMotorSimple.Direction.REVERSE
//                motorBL!!.direction = DcMotorSimple.Direction.REVERSE
//            }

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
                        drive1.poseEstimate = Pose2d(47.44, -60.20, Math.toRadians(90.0))
                    }

                    if (controller1.dpad_right && !(previousController1.dpad_right)) {
                        val teleopBasket = drive1.trajectorySequenceBuilder(drive1.poseEstimate)
                            .setReversed(true)
                            .lineToLinearHeading(basketPose)
                            .setReversed(false)
                            .build()
                        drive1.followTrajectorySequenceAsync(teleopBasket)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                    if (controller1.dpad_left && !previousController1.dpad_left) {
                        val teleopBar =  drive1.trajectorySequenceBuilder(drive1.poseEstimate)
                            .lineToLinearHeading(barPose)
                            .build()
                        drive1.followTrajectorySequenceAsync(teleopBar)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                    if (controller1.dpad_down && !previousController1.dpad_down) {
                        val teleopPickup = drive1.trajectorySequenceBuilder(drive1.poseEstimate)
                            .setReversed(true)
                            .splineToConstantHeading(clipPickVector, Math.toRadians(-90.00))
                            .setReversed(false)
                            .build()
                        drive1.followTrajectorySequenceAsync(teleopPickup)
                        automatedMovementToggle = AutomaticMovementState.Auto
                    }
                }
                AutomaticMovementState.Auto ->{
                    if (controller1.dpad_left && !previousController1.dpad_left) { drive1.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (controller1.dpad_right && !previousController1.dpad_right) { drive1.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (controller1.dpad_down && !previousController1.dpad_down) { drive1.breakFollowing();automatedMovementToggle = AutomaticMovementState.Manual }
                    if (!drive1.isBusy) { automatedMovementToggle = AutomaticMovementState.Manual }
                }
            }

            when (automaticTransferToggle) {
                AutomaticTransferState.Manual -> {
                    // Auto transfer toggle
                    if (controller2.left_bumper && !(previousController2.left_bumper)) {
                        automaticTransferToggle = AutomaticTransferState.StartTransfer
                    }
                    if (controller1.b && !previousController1.b){
                        transferSide = !transferSide
                    }
                    if (transferSide){ transferSideString = "Back Side" }
                    else {transferSideString = "Front Side"}
                    telemetry.addData("Transfer Side: ", transferSideString)

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
                    if (intakeInToggle) { intakeMotor?.power = 0.8 }
                    else if (intakeOutToggle) { intakeMotor?.power = -0.8 }
                    else { intakeMotor?.power = 0.0 }

                    //IN STOP SERVO
                    if (slideHorizontal!!.currentPosition < 200){ inStopServo!!.position = inStopAutoOpen }
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
                    }
                    if (controller1.x && !previousController1.x){
                        verticalSlideTo(verticalSlideBar, 1.0)
                        verticalHeight = verticalSlideBar
                        outSwivelServo!!.position = outSwivelPerpFront
                        outRotationServo!!.position = outRotationUp
                    }

                    //OUT ROTATION SERVO
                    if (controller2.b && !previousController2.b){
                        if (outRotation) {
                            verticalSlideTo(verticalSlideBar,1.0)
                            verticalHeight = verticalSlideBar
                            outRotationServo!!.position = outRotationUp
                            outSwivelServo!!.position = outSwivelPerpFront
                            outRotation = false
                        } else {
                            verticalSlideTo(50,1.0)
                            verticalHeight = 50
                            outRotationServo!!.position = outRotationBackWall
                            outSwivelServo!!.position = outSwivelPerpBack
                            inRotationToggle = false
                            outRotation = true
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
                    if (controller2.dpad_down && !previousController2.dpad_down) { verticalSlideToggle = VerticalSlideState.Floor; outRotationServo!!.position = outRotationCenter }
                    if (controller2.right_stick_button && !previousController2.right_stick_button) { verticalSlideToggle = VerticalSlideState.Floor; outRotationServo!!.position = outRotationCenter }
                    if (controller2.dpad_left && !previousController2.dpad_left) { verticalSlideToggle = VerticalSlideState.Bar; outRotationServo!!.position = outRotationUp }
                    if (rightY2!! >= 0.2 || rightY2!! <= -0.2) { verticalSlideToggle = VerticalSlideState.Manual }
                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2!! > 0) {
                                verticalSlideTo(4000, (rightY2 as Double) / 1.5)
                            } else if (rightY2!! < 0 && slideVertical!!.currentPosition > 0) {
                                verticalSlideTo(0, -(rightY2 as Double) / 1.5)
                            }
                            // This part here is for holding power
                            else if (controller2.right_stick_y.toDouble() == 0.0 && previousController2.right_stick_y.toDouble() != 0.0) {
                                verticalHeight = slideVertical!!.currentPosition
                                verticalSlideTo(verticalHeight, 1.0)
                            } else {
                                verticalSlideTo(verticalHeight, 1.0)
                            }
                        }
                        VerticalSlideState.Floor -> { verticalSlideTo(0,1.0);    verticalBackToManual() }
                        VerticalSlideState.Low   -> { verticalSlideTo(verticalSlideLow,1.0); verticalBackToManual() }
                        VerticalSlideState.High  -> { verticalSlideTo(verticalSlideHigh,1.0); verticalBackToManual() }
                        VerticalSlideState.Bar  -> { verticalSlideTo(verticalSlideBar,1.0); verticalBackToManual() }
                    }
                }

                //TRANSFER SYSTEM
                AutomaticTransferState.StartTransfer-> {
                    outRotationServo!!.position = outRotationCenter
                    outClawServo!!.position = outClawOpen
                    inRotationServo!!.position = inRotationTransfer
                    intakeMotor?.power = 0.7
                    horizontalSlideTo(0,0.8)
                    verticalSlideTo(40, 1.0)
                    transferServo!!.position = transferServoClose
                    outSwivelServo!!.position = outSwivelPerpBack
                    if (!doOnce) {
                        elapsedTime.reset()
                        doOnce = true
                        timeHor = slideHorizontal!!.currentPosition * 0.0005
                    }
                    if (elapsedTime.time() > timeHor - 0.3){
                        inStopServo!!.position = inStopAutoOpen
                    }
                    if ((slideVertical!!.currentPosition < 50) && (elapsedTime.time() > timeHor + 0.6)){
                        automaticTransferToggle = AutomaticTransferState.Pickup
                        doOnce = false
                    }
                }
                AutomaticTransferState.Pickup -> {
                    outClawServo!!.position = outClawClose
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 0.3) {
                        automaticTransferToggle = AutomaticTransferState.ResetSlide
                        doOnce = false
                    }
                }
                AutomaticTransferState.ResetSlide ->{
                    intakeMotor!!.power = 0.0
                    verticalSlideTo(verticalSlideHigh,1.0)
                    verticalHeight = verticalSlideHigh
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 0.15) {
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