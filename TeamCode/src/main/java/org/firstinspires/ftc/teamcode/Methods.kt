package org.firstinspires.ftc.teamcode

import android.util.Base64
import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.NormalizedColorSensor
import com.qualcomm.robotcore.hardware.NormalizedRGBA
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.drive.advanced.SampleMecanumDriveCancelable
import kotlin.math.abs


abstract class Methods : LinearOpMode() {
    enum class VerticalSlideState { Floor, Low, High, Manual, Bar }
    enum class HorizontalSlideState { Floor, Extend, Manual }
    enum class AutomaticTransferState { Manual, StartTransfer, Pickup, ResetSlide, RotateOut }
    enum class AutomaticMovementState { Manual, Auto }
    enum class ColorStates {None, Red, Yellow, Blue}
    enum class PipelineType { Red, Orange, Blue, AprilTag }
    //enum class HangStates { Up, Down, Reset, None}

    val transferServoClose = 0.73 //
    val transferServoOpen = 0.5 //

    val outClawClose = 0.36 //
    val outClawOpen = 0.21 //
    val outClawOpenAuto = 0.21

    val outSwivelPerpBack = 0.85//
    val outSwivelPerpFront = 0.18 //

    var outRotationBackPlace = 0.1
    var outRotationBackWall = 0.16
    var outRotationBackOut = 0.25
    var outRotationUp = 0.45
    var outRotationFrontOut = 0.65
    var outRotationFrontWall = 0.72
    var outRotationFrontPlace = 0.8
    val outRotationCenter = 0.97

    val inRotationPick = 0.81
    val inRotationUpAuto = 0.52
    val inRotationTransfer = 0.27
    val inRotationUp = 0.03

    val inStopClose = 0.86
    val inStopAutoOpen = 0.6
    val inStopOpen = 0.6

    val verticalSlideHigh = 2000
    val verticalSlideBar = 480
    val verticalSlideLow = 800
    val horizontalSlideExtend = 950

    var outClawToggle = false
    var inClawToggle = false
    var intakeInToggle = false
    var intakeOutToggle = false
    var inRotationToggle = false
    var outSwivelToggle = false
    var transferServoToggle = false
    var moveOn = false
    var doOnce = false
    var verticalHeight = 0
    var speedDiv = 2.3

    val elapsedTime = ElapsedTime()

    var colors: NormalizedRGBA? = null
    var startingColor = "blue"
    var colorSeen = "none"

    var drive: SampleMecanumDriveCancelable? = null
    val basketVector = Vector2d(-60.0, -59.0)
    val barVector = Vector2d(0.0, -31.3)
    val clipPickVector = Vector2d(47.44, -63.2)
    val basketPose = Pose2d(-59.0, -61.0, Math.toRadians(225.00))
    val barPose = Pose2d(0.0, -30.5, Math.toRadians(90.0))
    val clipPickPose = Pose2d(47.44, -62.0, Math.toRadians(90.0))

    var horizontalSlideToggle = HorizontalSlideState.Manual
    var verticalSlideToggle = VerticalSlideState.Manual
    var automaticTransferToggle = AutomaticTransferState.Manual
    var automatedMovementToggle = AutomaticMovementState.Manual
    var currentColor = ColorStates.None
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
    var colorSensor : NormalizedColorSensor? = null

    // Places the sample

    fun spitOut(milliseconds : Long){
        intakeMotor!!.power = -0.3
        sleep(milliseconds)
        intakeMotor!!.power = 0.0
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

    fun initServosAndSensorsNoSet() {
        transferServo = hardwareMap.servo["Transfer"]
        outClawServo = hardwareMap.servo["OutClaw"]
        outRotationServo = hardwareMap.servo["OutRotation"]
        outSwivelServo = hardwareMap.servo["OutSwivel"]
        inStopServo = hardwareMap.servo["InStop"]   //4ex
        inRotationServo = hardwareMap.servo["InRotation"] //5ex
        colorSensor = hardwareMap.get(NormalizedColorSensor::class.java, "color")
        colorSensor!!.gain = 50.0F
    }

    //Initializes and sets motors but does not reset the motors
    fun initMotorsNoReset() {
        motorFL = hardwareMap.dcMotor["motorFL"] //1co
        motorFR = hardwareMap.dcMotor["motorFR"] //3ex  odX
        motorBL = hardwareMap.dcMotor["motorBL"] //2co  odY
        motorBR = hardwareMap.dcMotor["motorBR"] //2ex
        slideVertical = hardwareMap.dcMotor["verticalSlide"]  //0co
        slideHorizontal = hardwareMap.dcMotor["horizontalSlide"]  //0ex
        intakeMotor = hardwareMap.dcMotor["intakeMotor"]   //1ex
        setMotorModePositionNoReset(slideHorizontal!!)
        setMotorModePositionNoReset(slideVertical!!)
        setMotorModeEncoderNoReset(intakeMotor!!)
        motorFL!!.direction = DcMotorSimple.Direction.REVERSE
        motorBL!!.direction = DcMotorSimple.Direction.REVERSE
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

    // Uploads a pipeline to the robot
    fun switchPipeline(limelight: Limelight3A, data: String) {
        val decodedData = Base64.decode(data, 0).toString()

        //limelight.uploadPipeline(decodedData, 0)
        limelight.pipelineSwitch(0) // Just in case
    }

    // Uploads a pipeline specified by an enum to the robot
    fun switchPipelineEnum(limelight: Limelight3A, enum: PipelineType) {
        val fileName = when (enum) {
            PipelineType.Red -> LimeLightPipelines.RedPipeline
            PipelineType.Blue -> LimeLightPipelines.BluePipeline
            PipelineType.Orange -> LimeLightPipelines.OrangePipeline
            PipelineType.AprilTag -> LimeLightPipelines.AprilTagPipeline
        }

        switchPipeline(limelight, fileName)
    }

    fun formatSnapScript(colorLower: List<Int>, colorUpper: List<Int>): String {
        assert(colorLower.size == 3)
        assert(colorUpper.size == 3)

        val ss = LimeLightPipelines.SnapScript

        // Good enough, but could be better
        ss.replace("cl0", colorLower[0].toString())
        ss.replace("cl1", colorLower[1].toString())
        ss.replace("cl2", colorLower[2].toString())
        ss.replace("cu0", colorUpper[0].toString())
        ss.replace("cu1", colorUpper[1].toString())
        ss.replace("cu2", colorUpper[2].toString())

        return ss
    }

    //insideJokes
    fun insideJokes() {
        telemetry.addLine(when ((0..99).random()) {
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
            54 -> "You should be able to undertake and complete anything you desire"
            55 -> "How purple is it?"
            56 -> "There is not a single person on the earth who loves you"
            57 -> "On what spectrum?"
            58 -> "We did it lasterday"
            59 -> "That is flagrantly upside down"
            60 -> "I literally just stand here and I look at you guys and think god when is this done"
            61 -> "I’m sorry that I want the robot to make it past the halfway point of the field."
            62 -> "Ok again I don’t know what sine means"
            63 -> "The sandbag/build fail of 2023"
            64 -> "All fixed"
            65 -> "Knot you"
            66 -> "But like setting off an EMP could be kind of fun though. What if we did it in the middle of Chicago."
            67 -> "The thumb’s not a finger, so…"
            68 -> "They should be singing in the closet"
            69 -> "An actual quote that’s worthy of being quoted"
            70 -> "Intake the baby"
            71 -> "Antifluent"
            72 -> "Fire in the hole"
            73 -> "1 hour of silence broken up with dodgeball sounds"
            74 -> "As bad as the cabinet is, the codebase is worse"
            75 -> " *growls* damn it Isaac!"
            76 -> "Us want to kick you off the team"
            77 -> "The car is turning left"
            78 -> "Get an actual image"
            79 -> "Does he have a sailor’s mouth?"
            80 -> "Shove it down my throat"
            81 -> "Not everything has a sexual connotation Isaac"
            82 -> "His standards are not your standards"
            83 -> "Ok Pumpkin"
            84 -> "I’ve known bats that can see better than you"
            85 ->  "I created you and I can destroy you"
            86 -> "You need to start making age appropriate jokes"
            87 -> "Cavalier with his usage"
            88 -> "Stop covering your ass no one’s gonna force you to wipe"
            89 -> "Beware the Rohan G. Bias"
            90 -> "From the cookies"
            91 -> "We’ll never have to do that again"
            92 -> "Burger, no onions, with a side of love"
            93 -> "Martin Luther King Boulevard Jr"
            94 -> "It never jiggled again 😢"
            95 -> "He’s to busy agreeing"
            96 -> "Yeah, but you down mean it"
            97 -> "I don’t know, whichever one rolls"
            98 -> "Are we really going to eat all of his zip ties?"
            else -> "Why did we add these?"
        })
        telemetry.update()
    }
}