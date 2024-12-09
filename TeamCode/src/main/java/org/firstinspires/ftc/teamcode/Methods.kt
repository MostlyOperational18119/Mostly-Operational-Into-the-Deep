package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import kotlin.math.abs


abstract class Methods : LinearOpMode() {
    enum class VerticalSlideState { Floor, Low, High, Manual, Bar }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Pickup, Transfer }
    enum class AutomaticMovementState { Manual, Auto }
    enum class  HangStates { Up, Down, Reset, None}

    val clawRotateRest = 0.63
    val clawRotateUpRight = 0.48
    val clawRotateOut = 0.0
    val clawRotateStraight = 0.08
    val clawRotateWall = 0.16
    val transferDownPos = 0.57
    val servoHangActive = 0.44
    val servoHangPassive = 0.3
    val transferUpPos = 0.18
    val clawServoOpen = 0.1
    val clawServoClosed = 0.22

    val verticalSlideLow = 2300
    val verticalSlideHigh = 3650
    val verticalSlideBar = 1738
    val horizontalSlideExtend = 1600
    val speedDiv = 2.3
    var singleRunCheck = 1

    var moveRotateServo = false
    var intakeInToggle = false
    var intakeOutToggle = false

    val basketVector = Vector2d(-58.26, -57.64)
    val basketHeading = Math.toRadians(225.00)
    val barVector = Vector2d(-10.04, -34.01)
    val barHeading = Math.toRadians(-90.00)
    val basketPose = Pose2d(-58.26, -57.64, 225.0)
    val barPose = Pose2d(-10.04, -34.01, -90.0)

    var horizontalSlideToggle = HorizontalSlideState.Manual
    var verticalSlideToggle = VerticalSlideState.Manual
    var automatedTransferToggle = AutomaticTransferState.Pickup
    var automatedMovementToggle = AutomaticMovementState.Manual
    var hangerState = HangStates.None

    val controller1 = Gamepad()
    val controller2 = Gamepad()
    val previousController1 = Gamepad()
    val previousController2 = Gamepad()

    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var slideVerticalMotor: DcMotor? = null
    var slideHorizontalMotor: DcMotor? = null
    var hangerMotor: DcMotor? = null
    var tapeMeasureRotateMotor: DcMotor? = null
    var intakeServo : CRServo? = null
    var clawServo : Servo? = null
    var transferServo : Servo? = null
    var clawRotateServo : Servo? = null
    var hangPusher : Servo? = null
    var hangTouch : TouchSensor? = null

    //Intake the pixel with a sleep
    fun intakePixel (sleep: Long){
        intakeServo!!.power = 1.0
        horizontalSlideTo(800, 1.0)
        sleep(sleep)
    }

    //places the sample. Really just opening the claw
    fun placeSample(){
        sleep(200)
        clawServo!!.position = clawServoOpen
        sleep(200)
    }

    //Sets the position of verical motor and moves. Only uses 1 power since it checks.
    fun verticalSlideTo(position : Int, power : Double){
        slideVerticalMotor!!.targetPosition = position
        if (slideVerticalMotor!!.currentPosition < slideVerticalMotor!!.targetPosition) {
            slideVerticalMotor!!.power = power
        }
        else{
            slideVerticalMotor!!.power = -power
        }
    }

    //Sets the position of horizontal motor and moves. Only uses 1 power since it checks.
    fun horizontalSlideTo(position : Int, power : Double){
        slideHorizontalMotor!!.targetPosition = position
        if (slideHorizontalMotor!!.currentPosition < slideHorizontalMotor!!.targetPosition) {
            slideHorizontalMotor!!.power = power
        }
        else{
            slideHorizontalMotor!!.power = -power
        }
    }

    //Resets the mode back to Manual for Vertical Motor in Vertical States
    fun verticalBackToManual(){
        if (abs(slideVerticalMotor!!.currentPosition - slideVerticalMotor!!.targetPosition) < 20) {
            verticalSlideToggle = VerticalSlideState.Manual
        }
    }

    //Resets the mode back to Manual for Horizontal Motor in Horizontal States
    fun horizontalBackToManual(){
        if (abs(slideHorizontalMotor!!.currentPosition - slideHorizontalMotor!!.targetPosition) < 20) {
            horizontalSlideToggle = HorizontalSlideState.Manual
        }
    }

    //The slow but consistent transfer when the slide is already down
    fun transferSlowDown (){
        intakeServo!!.power = 0.0
        horizontalSlideTo(0,1.0)
        clawRotateServo!!.position = clawRotateUpRight
        verticalSlideTo(400,1.0)
        transferServo!!.position = transferUpPos
        clawServo!!.position = clawServoOpen
        clawRotateServo!!.position = clawRotateRest
        sleep(500)
        verticalSlideTo(0,1.0)
        sleep(500)
        clawServo!!.position = clawServoClosed
        sleep(500)
        verticalSlideTo(1000,1.0)
        clawRotateServo!!.position = clawRotateOut
        sleep(1500)
        transferServo!!.position = transferDownPos
    }

    //Copies the Controls for Teleop
    fun copyControls (){
        previousController1.copy(controller1)
        previousController2.copy(controller2)
        controller1.copy(gamepad1)
        controller2.copy(gamepad2)
    }

    //Initializes the Servos and Touch Sensor
    fun initServosAndTouch(){
        intakeServo = hardwareMap.crservo["intakeServo"]
        clawServo = hardwareMap.servo["clawServo"]
        clawServo!!.position =  clawServoClosed
        transferServo = hardwareMap.servo["transferServo"]
        transferServo!!.position = transferUpPos
        clawRotateServo = hardwareMap.servo["rotateServo"]
        clawRotateServo!!.position = clawRotateUpRight
        hangPusher = hardwareMap.servo["hangPusher"]
        hangTouch = hardwareMap.touchSensor["HangTouch"]
    }

    //Initializes and sets motors but does not reset the motors
    fun initMotorsNoReset (){
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        hangerMotor = hardwareMap.dcMotor["hanger"]
        tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        motorBR!!.direction = DcMotorSimple.Direction.REVERSE
        motorFR!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePositionNoReset(slideVerticalMotor!!)
        setMotorModePositionNoReset(hangerMotor!!)
        slideVerticalMotor!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor!!)
        slideHorizontalMotor!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoderNoReset(tapeMeasureRotateMotor!!)
        setMotorModeEncoderNoReset(hangerMotor!!)
    }

    //Initializes and sets motors amd resets them
    fun initMotors(){
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        hangerMotor = hardwareMap.dcMotor["hanger"]
        tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        motorBR!!.direction = DcMotorSimple.Direction.REVERSE
        motorFR!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideVerticalMotor!!)
        setMotorModePosition(hangerMotor!!)
        slideVerticalMotor!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModePosition(slideHorizontalMotor!!)
        slideHorizontalMotor!!.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(tapeMeasureRotateMotor!!)
        setMotorModeEncoder(hangerMotor!!)
    }

    //Sets the mode of a motor to encoder with reset
    fun setMotorModeEncoder(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    //Sets the mode of a motor to position with reset
    fun setMotorModePosition(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    //Sets the mode of a motor to encoder without reset
    fun setMotorModeEncoderNoReset(motor: DcMotor) {
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    //Sets the mode of a motor to position without reset
    fun setMotorModePositionNoReset(motor: DcMotor) {
        motor.targetPosition = 0
        motor.mode = DcMotor.RunMode.RUN_TO_POSITION
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
    }

    //insideJokes
    fun insideJokes (){
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
    }
}