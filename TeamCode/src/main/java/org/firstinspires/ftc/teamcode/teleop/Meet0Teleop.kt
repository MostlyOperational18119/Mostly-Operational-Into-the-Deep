package org.firstinspires.ftc.teamcode.teleop

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.Servo
import kotlin.math.PI
import kotlin.math.cos

@TeleOp(name = "MEET 1 TELEOP (\uD83D\uDC37)", group = "AAAAAA")
class Meet0Teleop : LinearOpMode() {
    override fun runOpMode () {
        telemetry.addData("Status", "Initialized")
        telemetry.update()

        //VARIABLES
        var rotateTarget = 0
        var slideTarget = 0
        var speedDiv = 2
        val rotateServoMid = 0.11
        val rotateServoRight = 0.0
        val rotateServoLeft = 0.25
        var rotateDiv = 3
        var slideDiv = 1.5
        var slideInches = 12.0
        var angle = 0.0
        var slideHorLength = 0.0

        //TOGGLES
        var rightintakeToggle = false
        var leftintakeToggle = false
//        var autoRotateUpToggle = false
//        var autoSlideDownToggle = false
//        var autoSlideUpToggle = false

        //GAME PADS
        val currentGamepad1 = Gamepad()
        val currentGamepad2 = Gamepad()
        val previousGamepad1 = Gamepad()
        val previousGamepad2= Gamepad()

        //Defining Motors/Servos
        val FL = hardwareMap.get(DcMotor::class.java, "motorFL")
        val BL = hardwareMap.get(DcMotor::class.java, "motorBL")
        val FR = hardwareMap.get(DcMotor::class.java, "motorFR")
        val BR = hardwareMap.get(DcMotor::class.java, "motorBR")
        BR.direction = DcMotorSimple.Direction.REVERSE
        FR.direction = DcMotorSimple.Direction.REVERSE

        val rotateMotor = hardwareMap.get(DcMotor::class.java, "motorRotate")
        rotateMotor.targetPosition = 0
        rotateMotor.power = 0.0
        rotateMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        rotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val slideMotor = hardwareMap.get(DcMotor::class.java, "motorSlide")
        slideMotor.targetPosition = 0
        slideMotor.power = 0.0
        slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val clawServo = hardwareMap.get(CRServo::class.java, "clawServo")
        val rotateServo = hardwareMap.get(Servo::class.java, "rotateServo")

        waitForStart()

        while(opModeIsActive()) {
            //GAMEPADS
            previousGamepad1.copy(currentGamepad1)
            previousGamepad2.copy(currentGamepad2)
            currentGamepad1.copy(gamepad1)
            currentGamepad2.copy(gamepad2)

            //INPUT
            val y = -gamepad1.left_stick_y.toDouble() // Remember, Y stick is reversed!
            val x = gamepad1.left_stick_x.toDouble()
            val rx = gamepad1.right_stick_x.toDouble()
            val lj = gamepad2.left_stick_y.toDouble()
            val rj = gamepad2.right_stick_y.toDouble()

            //RESET MOTORS
            //Basically if you click b then everything will reset the encoders to current pos.
            if (currentGamepad1.b&& !previousGamepad1.b) {
                rotateMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                rotateMotor.targetPosition = 0
                rotateMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                rotateMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

                slideMotor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
                slideMotor.targetPosition = 0
                slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
                slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            }

            //MOVEMENT
            FL.power = (y + x + rx)/speedDiv
            BL.power = (y - x + rx)/speedDiv
            FR.power = (y - x - rx)/speedDiv
            BR.power = (y + x - rx)/speedDiv

            //SPEED DIV CHANGING
            //If they click certain buttons then the speed div will change.
            if (gamepad1.left_trigger > 0.1){
                speedDiv = 3
            }
            else if (gamepad1.right_trigger > 0.1){
                speedDiv= 1
            }
            else {
                speedDiv = 2
            }

            //TRIG
            //Trigonometry for getting the length of the horizontal slide.

            //This gets length of slide
            slideInches = -slideMotor.currentPosition / 100.0 + 13.5
            //This gets angle in degrees
            angle = 90 - ((90/800.0)*rotateMotor.currentPosition)
            //Changes it to radians
            angle = angle * Math.PI / 180
            //Uses cos time hypotenuse to get length of horizontal.
            slideHorLength = cos(Math.abs(angle)) * slideInches +5

            //CLAW SERVO
            //If you click either bumper, it will turn off the other and change the one that you are on
            //This allows for easy toggling between the 3 states.
            if (currentGamepad2.right_bumper&& !previousGamepad2.right_bumper){
                rightintakeToggle = !rightintakeToggle
                leftintakeToggle = false
            }
            if (currentGamepad2.left_bumper&& !previousGamepad2.left_bumper){
                leftintakeToggle = !leftintakeToggle
                rightintakeToggle = false
            }

            //Uses the toggles made before to change the power of the claw servo
            if (rightintakeToggle) {
                clawServo?.power = 1.0
            }
            else if (leftintakeToggle) {
                clawServo?.power = -1.0
            }
            else { clawServo?.power = 0.0 }

            //ROTATE SERVO

            //the rotate servo only moves when the slide encoder if more than certain threshold since it will hit slide
            if (slideMotor.currentPosition > -500){
                rotateServo.position = rotateServoMid
            }
            //For each button, it will just set the positon of the servo to the preset positions.
            else if (currentGamepad2.b&& !previousGamepad2.b) {
                rotateServo.position = rotateServoRight
            }
            else if (currentGamepad2.x&& !previousGamepad2.x) {
                rotateServo.position = rotateServoLeft
            }
            //If the driver wants to tweak the angle they can do so.
            else if (currentGamepad2.dpad_left&& !previousGamepad2.dpad_left) {
                rotateServo.position+=0.1
            }
            else if (currentGamepad2.dpad_right&& !previousGamepad2.dpad_right) {
                rotateServo.position-=0.1
            }
            else if (currentGamepad2.a&& !previousGamepad2.a){
                rotateServo.position = rotateServoMid
            }

//            if (currentGamepad1.x&& !previousGamepad1.x) {
//                autoRotateUpToggle = true
//            }

            //ROTATE

            //If the horizontal length of the slides is greater than -3, so it is not backward, then we can move the rotation back
            //This sets the target to 0, but only moves when we give the power.
            if (rj>0 && slideHorLength > -3) {
                rotateMotor.targetPosition = 0
                rotateMotor.power = rj/rotateDiv
                rotateTarget = 0
            }
            //If the horizontal length of the slides is less than 42, so it is not too forward, then we can move the rotation forward
            //This sets the target to 1450, but only moves when we give the power.
            else if (rj<0 && slideHorLength < 42) {
                rotateMotor.targetPosition = 1450
                rotateMotor.power = -rj/rotateDiv
                rotateTarget = 0
            }
            //Makes sure that the rotation does not move when we move the joysticks.
            else {
                if (rotateTarget == 0) {
                    rotateMotor.targetPosition = rotateMotor.currentPosition
                    rotateTarget = rotateMotor.currentPosition
                }
            }

            //ROTATE SPEED CHANGING
            //If you click the button, the rotation div will change.
            if (gamepad1.right_trigger > 0.1) {
                rotateDiv = 5
            }else {
                rotateDiv = 3
            }

//            if (currentGamepad1.y&& !previousGamepad1.y) {
//                autoSlideDownToggle = true
//            }
//            if (currentGamepad1.a&& !previousGamepad1.a) {
//                autoSlideUpToggle = true
//            }

            //SLIDES
            //If the horizontal length of the slides is greater than -2 and less than 42
            //It is not too far extended so we can further extend.
            //This sets the target to -4100, but only moves when we give the power.
            if (lj<0 && slideHorLength <= 42 && slideHorLength >= -2) {
                slideMotor.targetPosition = -4100
                slideMotor.power = -lj/slideDiv
                slideTarget = 0
            }
            //If you move the slide down, then it will lower. The negativity of power does not matter.
            //This sets the target to 0, but only moves when we give the power.
            else if (lj>0) {
                slideMotor.targetPosition = 0
                slideMotor.power = lj/slideDiv
                slideTarget = 0
            }
            //Makes sure that the rotation does not move when we move the joysticks.
            else {
                if (slideTarget == 0) {
                    slideMotor.targetPosition = slideMotor.currentPosition
                    slideTarget = slideMotor.currentPosition
                }
            }

            //IDK WTF ALEX DID THIS FOR
            if (angle >= -20 && angle <= 5 && slideHorLength > 35) {
                slideMotor.targetPosition = 0
                slideMotor.power = lj/slideDiv
            }

            //Changing speed div for slides based on input from drivers.
            if (gamepad2.right_trigger > 0.1){
                slideDiv = 1.0
            } else {
                slideDiv = 1.5
            }

            //TELEMETRY
            telemetry.addData("BL Power: ",  BL.power)
            telemetry.addData("BR Power:  ",  BR.power)
            telemetry.addData("FL Power: ",  FL.power)
            telemetry.addData("FR Power:  ",  FR.power)
            telemetry.addData("RotateServo Postion:  ",  rotateServo.position)
            telemetry.addData("Slide Encoder Position: ",  slideMotor?.let { (it.currentPosition)})
            telemetry.addData("Angle (Degree): ", angle * 180.0/ PI )
            telemetry.addData("Slide Length (IN): ",  slideInches)
            telemetry.addData("Slide Horizontal Inches: ",  slideHorLength)
            telemetry.addData("Rotate Encoder Position: ",  rotateMotor?.let { (it.currentPosition)})
            telemetry.addData("Rotate Power: ",  rotateMotor?.let { (it.power)})
            telemetry.addLine("OpMode is active")
            telemetry.update()
        }
    }
}