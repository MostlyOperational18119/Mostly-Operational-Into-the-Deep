package org.firstinspires.ftc.teamcode

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.SampleGoBildaPinpointMecanumDriveCancelable
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import kotlin.math.abs


abstract class Methods : LinearOpMode() {
    enum class VerticalSlideState { Floor, Low, High, Manual, AutoPlaceTransfer }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Manual, StartTransfer, Pickup, ResetSlide, RotateOut }
    enum class AutomaticMovementState { Manual, Auto }
    //enum class HangStates { Up, Down, Reset, None}

    val transferServoClose = 0.53
    val transferServoOpen = 0.84
    val outClawClose = 0.62
    val outClawOpen = 0.35
    val outSwivelPerpBack = 0.23
    val outSwivelPerpFront = 0.9 // Not really necessary anymore
    val outRotationBack = 0.26
    val outRotationCenter = 0.59 // "center"
    val outRotationFront = 0.84
    val inRotationPick = 0.64
    val inRotationDormant = 0.8
    val inRotationTransfer = 0.98
    val inStopClose = 0.00
    val inStopOpen = 0.36

    val verticalSlideWall = 500
    val verticalSlideHigh = 3500
    val verticalSlideBar = 1500
    val horizontalSlideExtend = 950

    var outClawToggle = false
    var inClawToggle = false
    var intakeInToggle = false
    var intakeOutToggle = false
    var inRotationToggle = false
    var outSwivelToggle = false
    var transferServoToggle = false
    var doOnce = false
    var verticalHeight = 0
    var speedDiv = 2.3

    val elapsedTime = ElapsedTime()

    var drive: SampleMecanumDriveCancelable? = null
    val basketVector = Vector2d(-58.26, -57.64)
    val basketHeading = Math.toRadians(225.00)
    val barVector = Vector2d(-10.04, -34.01)
    val barHeading = Math.toRadians(-90.00)
    val basketPose = Pose2d(-52.5, -52.5, 45.0)
    val barPose = Pose2d(-10.04, -42.15, 90.0)

    var horizontalSlideToggle = HorizontalSlideState.Manual
    var verticalSlideToggle = VerticalSlideState.Manual
    var automaticTransferToggle = AutomaticTransferState.Manual
    var automatedMovementToggle = AutomaticMovementState.Manual
//    var hangerState = HangStates.None

    val controller1 = Gamepad()
    val controller2 = Gamepad()
    val previousController1 = Gamepad()
    val previousController2 = Gamepad()

    var leftY1: Double? = null
    var leftX1: Double? = null
    var rightX1: Double? = null
    var leftY2: Double? = null
    var rightY2: Double? = null

    var motorFL: DcMotor? = null
    var motorBL: DcMotor? = null
    var motorFR: DcMotor? = null
    var motorBR: DcMotor? = null
    var slideVertical: DcMotor? = null
    var slideHorizontal: DcMotor? = null
    var intakeMotor: DcMotor? = null

    var transferServo : Servo? = null
    var outClawServo : Servo? = null
    var outRotationServo : Servo? = null
    var outSwivelServo : Servo? = null
    var inStopServo : Servo? = null
    var inRotationServo : Servo? = null

    // Places the sample
    fun placeSample() {
        outClawServo!!.position = outClawOpen
        sleep(500)

        outClawServo!!.position = outClawClose
    }

    // Places the specimen
    fun placeSpecimen() {
        verticalSlideTo(750,1.0)
        sleep(400)
        outClawServo!!.position = outClawOpen
    }

    fun intakeSample() {
        horizontalSlideTo(horizontalSlideExtend, 0.5)
//        inClawServo!!.position = inClawOpen
        inRotationServo!!.position = inRotationPick

        sleep(300)

//        inClawServo!!.position = inClawClose
    }

    //Sets the position of vertical motor and moves. Only uses 1 power since it checks.
    fun verticalSlideTo(position : Int, power : Double) {
        slideVertical!!.targetPosition = position
        if (slideVertical!!.currentPosition < slideVertical!!.targetPosition) {
            slideVertical!!.power = power
        }
        else{
            slideVertical!!.power = -power
        }
    }

    //Sets the position of horizontal motor and moves. Only uses 1 power since it checks.
    fun horizontalSlideTo(position : Int, power : Double) {
        slideHorizontal!!.targetPosition = position
        if (slideHorizontal!!.currentPosition < slideHorizontal!!.targetPosition) {
            slideHorizontal!!.power = power
        }
        else{
            slideHorizontal!!.power = -power
        }
    }

    fun verticalBackToManual() {
        if (abs(slideVertical!!.currentPosition - slideVertical!!.targetPosition) < 40) {
            verticalSlideToggle = VerticalSlideState.Manual
            verticalHeight = slideVertical!!.currentPosition
        }
    }

    //Resets the mode back to Manual for Horizontal Motor in Horizontal States
    fun horizontalBackToManual() {
        if (abs(slideHorizontal!!.currentPosition - slideHorizontal!!.targetPosition) < 40) {
            horizontalSlideToggle = HorizontalSlideState.Manual
        }
    }

    //Initializes the Servos and Touch Sensor
    fun initServosAndTouchWithSet() {
        transferServo = hardwareMap.servo["Transfer"]
        outClawServo = hardwareMap.servo["OutClaw"]
        outClawServo!!.position = outClawClose
        outRotationServo = hardwareMap.servo["OutRotation"]
        outRotationServo!!.position = outRotationCenter
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inRotationServo = hardwareMap.servo["InRotation"]
    }

    fun initServosAndTouchWithoutSet() {
        transferServo = hardwareMap.servo["Transfer"]
        outClawServo = hardwareMap.servo["OutClaw"]
        outRotationServo = hardwareMap.servo["OutRotation"]
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]
        inRotationServo = hardwareMap.servo["InRotation"]
    }

    //Initializes and sets motors but does not reset the motors
    fun initMotorsNoReset() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVertical = hardwareMap.dcMotor["verticalSlide"]
        slideHorizontal = hardwareMap.dcMotor["horizontalSlide"]
        intakeMotor = hardwareMap.dcMotor["intakeMotor"]
        setMotorModePositionNoReset(slideHorizontal!!)
        setMotorModePositionNoReset(slideVertical!!)
        setMotorModeEncoderNoReset(intakeMotor!!)
        motorFL!!.direction = DcMotorSimple.Direction.REVERSE
        motorBL!!.direction = DcMotorSimple.Direction.REVERSE
        slideHorizontal!!.direction = DcMotorSimple.Direction.REVERSE
    }

    //Initializes and sets motors amd resets them
    fun initMotors() {
        motorFL = hardwareMap.dcMotor["motorFL"]
        motorFR = hardwareMap.dcMotor["motorFR"]
        motorBL = hardwareMap.dcMotor["motorBL"]
        motorBR = hardwareMap.dcMotor["motorBR"]
        slideVertical = hardwareMap.dcMotor["verticalSlide"]
        slideHorizontal = hardwareMap.dcMotor["horizontalSlide"]
        intakeMotor = hardwareMap.dcMotor["intakeMotor"]
        setMotorModeEncoder(intakeMotor!!)
        setMotorModePosition(slideHorizontal!!)
        setMotorModePosition(slideVertical!!)
        motorFL!!.direction = DcMotorSimple.Direction.REVERSE
        motorBL!!.direction = DcMotorSimple.Direction.REVERSE
        slideHorizontal!!.direction = DcMotorSimple.Direction.REVERSE
    }

    fun initOdometry() {
        drive = SampleMecanumDriveCancelable(hardwareMap)
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
    fun insideJokes() {
        telemetry.addLine(when ((0..54).random()) {
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
            50 -> "the west has fallen"
            51 -> "bring it back ho"
            52 -> "fallacy fallacy"
            53 -> "with a side of love"
            else -> "Why did we add these?"
        })
        telemetry.update()
    }
}