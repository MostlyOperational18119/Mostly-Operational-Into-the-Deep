package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.util.ElapsedTime
import kotlin.math.abs

@TeleOp(name = "The Brainiacs \uD83E\uDDE0", group = "ZZZ")
class NewBotTeleop: LinearOpMode() {
    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var slideVertical: DcMotor? = null
    var slideHorizontal: DcMotor? = null

    var leftY1: Double? = null
    var leftX1: Double? = null
    var rightX1: Double? = null
    var leftY2: Double? = null
    var rightY2: Double? = null

    val controller1 = Gamepad()
    val controller2 = Gamepad()
    val previousController1 = Gamepad()
    val previousController2 = Gamepad()

    val elapsedTime = ElapsedTime()

    enum class VerticalSlideState { Floor, Low, High, Manual, Bar }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Manual, StartTransfer, InRotate, SlideDown, Pickup, ResetSlide, RotateOut }

    var verticalSlideToggle = VerticalSlideState.Manual
    var automaticTransferToggle = AutomaticTransferState.Manual
    var horizontalSlideToggle = HorizontalSlideState.Manual

    val transferServoNormal = 0.84
    val transferServoIntake = 0.94
    val outClawClose = 0.56
    val outClawOpen = 0.38
    val outSwivelParallel = 0.11
    val outSwivelPerp = 0.44
    val outRotationBack = 0.1
    val outRotationCenter = 0.6
    val outRotationFront = 1.0
    val inRotationPick = 0.1
    val inRotationTransfer = 1.0
    val inSwivelRight = 0.6
    val inSwivelCenter = 0.8
    val inSwivelLeft = 1.0
    val inClawOpen = 0.5
    val inClawClose = 0.95

    var verticalHeight = 0

    override fun runOpMode() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVertical = hardwareMap.dcMotor["verticalSlide"]
        slideHorizontal = hardwareMap.dcMotor["horizontalSlide"]
        slideHorizontal!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideHorizontal!!.targetPosition = 0
        slideHorizontal!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideHorizontal!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        slideVertical!!.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        slideVertical!!.targetPosition = 0
        slideVertical!!.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideVertical!!.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motorBL!!.direction = DcMotorSimple.Direction.REVERSE
        motorFL!!.direction = DcMotorSimple.Direction.REVERSE
        slideHorizontal!!.direction = DcMotorSimple.Direction.REVERSE

        val transferServo = hardwareMap.servo["Transfer"]
        transferServo.position = transferServoNormal
        val outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo.position = outClawClose
        val outRotationServo = hardwareMap.servo["OutRotation"]
        outRotationServo.position = outRotationCenter
        val outSwivelServo = hardwareMap.servo["OutSwivel"]
        outSwivelServo.position = outSwivelParallel
        val inSwivelServo = hardwareMap.servo["InSwivel"]
        inSwivelServo.position = inSwivelCenter
        val inRotationServo = hardwareMap.servo["InRotation"]
        val inClawServo = hardwareMap.servo["InClaw"]

        var outClawToggle = false
        var inClawToggle = false
        var inRotationToggle = false
        var outSwivelToggle = false
        var transferServoToggle = false
        var doOnce = false

        waitForStart()

        while (opModeIsActive()) {
            leftY1 = -gamepad1.left_stick_y.toDouble()
            leftX1 = gamepad1.left_stick_x.toDouble()
            rightX1 = gamepad1.right_stick_x.toDouble()
            leftY2 = -gamepad2.left_stick_y.toDouble()
            rightY2 = -gamepad2.right_stick_y.toDouble()

            previousController1.copy(controller1)
            previousController2.copy(controller2)
            controller1.copy(gamepad1)
            controller2.copy(gamepad2)

            motorFL!!.power = (leftY1!! + leftX1!!+ rightX1!!)
            motorBL!!.power = (leftY1!! - leftX1!! + rightX1!!)
            motorFR!!.power = (leftY1!! - leftX1!! - rightX1!!)
            motorBR!!.power = (leftY1!! + leftX1!! - rightX1!!)

            when (automaticTransferToggle) {
                AutomaticTransferState.Manual -> {
                    //AUTO TRANSFER TOGGLE
                    if (controller1.right_trigger > 0.5 && !(previousController1.right_trigger > 0.5)) {
                        automaticTransferToggle = AutomaticTransferState.StartTransfer
                    }

                    //ALL SERVOS STUFF HERE. ASK BUILD FOR WHICH SERVO IS WHICH
                    if (controller1.a && !previousController1.a) { outClawToggle = !outClawToggle }
                    if (outClawToggle){ outClawServo.position = outClawOpen}
                    if (!outClawToggle){ outClawServo.position = outClawClose}

                    if (controller1.b && !previousController1.b) { inClawToggle = !inClawToggle }
                    if (inClawToggle){ inClawServo.position = inClawOpen}
                    if (!inClawToggle){ inClawServo.position = inClawClose}

                    if (controller1.x && !previousController1.x) { inRotationToggle = !inRotationToggle }
                    if (inRotationToggle){ inRotationServo.position = inRotationTransfer}
                    if (!inRotationToggle){ inRotationServo.position = inRotationPick}

                    if (controller1.y && !previousController1.y) { outSwivelToggle = !outSwivelToggle }
                    if (outSwivelToggle){ outSwivelServo.position = outSwivelParallel}
                    if (!outSwivelToggle){ outSwivelServo.position = outSwivelPerp}

                    if (controller1.left_trigger>0.5 && !(previousController1.left_trigger>0.5)) { transferServoToggle = !transferServoToggle }
                    if (transferServoToggle){ transferServo.position = transferServoIntake}
                    if (!transferServoToggle){ transferServo.position = transferServoNormal}

                    if (controller1.left_bumper && !previousController1.left_bumper) {
                        inSwivelServo.position +=0.02
                    }
                    if (controller1.right_bumper && !previousController1.right_bumper) {
                        inSwivelServo.position -=0.02
                    }

                    if (controller2.y && !previousController2.y) { horizontalSlideToggle = HorizontalSlideState.Floor }
                    if (controller2.x && !previousController2.x) { horizontalSlideToggle = HorizontalSlideState.Extend }
                    if (leftY2!! >= 0.2 || leftY2!! <= -0.2) { horizontalSlideToggle = HorizontalSlideState.Manual }
                    when (horizontalSlideToggle) {
                        HorizontalSlideState.Manual -> {
                            if (leftY2!! > 0 && slideHorizontal!!.currentPosition < 950) {
                                slideHorizontal!!.targetPosition = 950
                                slideHorizontal!!.power = leftY2 as Double
                            } else if (leftY2!! < 0 && slideHorizontal!!.currentPosition > 0) {
                                slideHorizontal!!.targetPosition = 0
                                slideHorizontal!!.power = leftY2 as Double
                            } else {
                                slideHorizontal!!.targetPosition = slideHorizontal!!.currentPosition
                                slideHorizontal!!.power = 0.1
                            }
                        }

                        HorizontalSlideState.Floor -> {
                            slideHorizontal!!.targetPosition = 0
                            if (slideHorizontal!!.currentPosition < slideHorizontal!!.targetPosition) {
                                slideHorizontal!!.power = 1.0
                            } else {
                                slideHorizontal!!.power = -1.0
                            }
                            if (abs(slideHorizontal!!.currentPosition - slideHorizontal!!.targetPosition) < 20) {
                                horizontalSlideToggle = HorizontalSlideState.Manual
                            }
                        }

                        HorizontalSlideState.Extend -> {
                            slideHorizontal!!.targetPosition = 950
                            if (slideHorizontal!!.currentPosition < slideHorizontal!!.targetPosition) {
                                slideHorizontal!!.power = 1.0
                            } else {
                                slideHorizontal!!.power = -1.0
                            }
                            if (abs(slideHorizontal!!.currentPosition - slideHorizontal!!.targetPosition) < 20) {
                                horizontalSlideToggle = HorizontalSlideState.Manual
                            }
                        }
                    }

                    if (controller2.dpad_up && !previousController2.dpad_up) {
                        verticalSlideToggle = VerticalSlideState.High
                    }
                    if (controller2.dpad_down && !previousController2.dpad_down) {
                        verticalSlideToggle = VerticalSlideState.Floor
                    }
                    if (controller2.dpad_left && !previousController2.dpad_left) {
                        verticalSlideToggle = VerticalSlideState.Low
                    }
                    if (controller2.dpad_right && !previousController2.dpad_right) {
                        verticalSlideToggle = VerticalSlideState.Bar
                    }
                    if (rightY2!! >= 0.2 || rightY2!! <= -0.2) {
                        verticalSlideToggle = VerticalSlideState.Manual
                    }

                    when (verticalSlideToggle) {
                        VerticalSlideState.Manual -> {
                            if (rightY2!! > 0 && slideVertical!!.currentPosition < 3700) {
                                slideVertical!!.targetPosition = 3700
                                slideVertical!!.power = (rightY2 as Double) / 3
                            } else if (rightY2!! < 0 && slideVertical!!.currentPosition > 0) {
                                slideVertical!!.targetPosition = 0
                                slideVertical!!.power = (rightY2 as Double) / 3
                            } else if (controller2.right_stick_y.toDouble() == 0.0 && previousController2.right_stick_y.toDouble() != 0.0) {
                                verticalHeight = slideVertical!!.currentPosition
                                slideVertical!!.targetPosition = verticalHeight
                                slideVertical!!.power = 0.3
                            } else {
                                slideVertical!!.targetPosition = verticalHeight
                                slideVertical!!.power = 0.3
                            }
                        }

                        VerticalSlideState.Floor -> {
                            slideVertical!!.targetPosition = 0
                            if (slideVertical!!.currentPosition < slideVertical!!.targetPosition) {
                                slideVertical!!.power = 1.0
                            } else {
                                slideVertical!!.power = -1.0
                            }
                            if (abs(slideVertical!!.currentPosition - slideVertical!!.targetPosition) < 50) {
                                verticalHeight = 0
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }

                        VerticalSlideState.Low -> {
                            slideVertical!!.targetPosition = 1000
                            if (slideVertical!!.currentPosition < slideVertical!!.targetPosition) {
                                slideVertical!!.power = 1.0
                            } else {
                                slideVertical!!.power = -1.0
                            }
                            if (abs(slideVertical!!.currentPosition - slideVertical!!.targetPosition) < 50) {
                                verticalHeight = 1000
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }

                        VerticalSlideState.High -> {
                            slideVertical!!.targetPosition = 3500
                            if (slideVertical!!.currentPosition < slideVertical!!.targetPosition) {
                                slideVertical!!.power = 1.0
                            } else {
                                slideVertical!!.power = -1.0
                            }
                            if (abs(slideVertical!!.currentPosition - slideVertical!!.targetPosition) < 50) {
                                verticalHeight = 3500
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }

                        VerticalSlideState.Bar -> {
                            slideVertical!!.targetPosition = 1500
                            if (slideVertical!!.currentPosition < slideVertical!!.targetPosition) {
                                slideVertical!!.power = 1.0
                            } else {
                                slideVertical!!.power = -1.0
                            }
                            if (abs(slideVertical!!.currentPosition - slideVertical!!.targetPosition) < 50) {
                                verticalHeight = 1500
                                verticalSlideToggle = VerticalSlideState.Manual
                            }
                        }
                    }

                    telemetry.addData("Vertical Current: ", slideVertical!!.currentPosition)
                    telemetry.addData("Vertical Target: ", slideVertical!!.targetPosition)
                    telemetry.addData("Vertical Power: ", slideVertical!!.power)
                    telemetry.addData("Hor Current: ", slideHorizontal!!.currentPosition)
                    telemetry.addData("Hor Target: ", slideHorizontal!!.targetPosition)
                    telemetry.addData("Hor Power: ", slideHorizontal!!.power)
                    telemetry.update()
                }

                AutomaticTransferState.StartTransfer-> {
                    inClawServo.position = inClawClose
                    inRotationServo.position = inRotationTransfer
                    slideHorizontal!!.targetPosition = 0
                    slideHorizontal!!.power = -1.0
                    slideVertical!!.targetPosition = 1200
                    slideVertical!!.power = 1.0
                    outRotationServo.position = outRotationCenter
                    outSwivelServo.position = outSwivelParallel
                    outClawServo.position = outClawOpen
                    transferServo.position = transferServoIntake
                    if (abs(1200 - slideVertical!!.currentPosition)  < 20){
                        inClawServo.position = inClawOpen
                        if (!doOnce) { elapsedTime.reset(); doOnce = true }
                        if (elapsedTime.time() > 1.0){
                            automaticTransferToggle = AutomaticTransferState.InRotate
                            doOnce = false
                        }
                    }
                }
                AutomaticTransferState.InRotate ->{
                    inRotationServo.position = inRotationPick
                    if (!doOnce) { elapsedTime.reset();doOnce = true }
                    if (elapsedTime.time() > 0.5) {
                        automaticTransferToggle = AutomaticTransferState.SlideDown
                        doOnce = false
                    }
                }

                AutomaticTransferState.SlideDown ->{
                    transferServo.position = transferServoNormal
                    slideVertical!!.targetPosition = 0
                    slideVertical!!.power = -1.0
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
                    slideVertical!!.targetPosition = 1500
                    slideVertical!!.power = 1.0
                    verticalHeight = 1500
                    if (!doOnce) { elapsedTime.reset(); doOnce = true }
                    if (elapsedTime.time() > 0.7) {
                        automaticTransferToggle = AutomaticTransferState.RotateOut
                        doOnce = false
                    }
                }
                AutomaticTransferState.RotateOut ->{
                    inRotationServo.position = inRotationPick
                    transferServo.position = transferServoNormal
                    outRotationServo.position = outRotationBack
                    outSwivelServo.position = outSwivelPerp
                    automaticTransferToggle = AutomaticTransferState.Manual
                }
            }
        }
    }
}