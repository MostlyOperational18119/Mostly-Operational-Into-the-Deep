package org.firstinspires.ftc.teamcode.autonomous

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.autonomous.PoseStorage
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous

@Autonomous(name = "BASKET_Meet2", group = "B")
class BASKET_MEET_2 : DriveMethods() {
    fun setMotorModeEncoder(motor: DcMotor) {
        motor.mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        motor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        motor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        motor.power = 0.0
    }

    override fun runOpMode() {
        // Setup Odometry :)
        val drive = SampleMecanumDrive(hardwareMap)

        // Motors
        val motorFL = hardwareMap.dcMotor["motorFL"]
        val motorFR = hardwareMap.dcMotor["motorFR"]
        val motorBL = hardwareMap.dcMotor["motorBL"]
        val motorBR = hardwareMap.dcMotor["motorBR"]
        val slideVerticalMotor = hardwareMap.dcMotor["slideVertical"]
        val slideHorizontalMotor = hardwareMap.dcMotor["slideHorizontal"]
        val tapeMeasureRotateMotor = hardwareMap.dcMotor["tapeMeasureRotateMotor"]

        // Set motor modes
        setMotorModeEncoder(slideVerticalMotor)
        slideVerticalMotor.direction = DcMotorSimple.Direction.REVERSE
        setMotorModeEncoder(slideHorizontalMotor)

        // Servos
        val clawServo = hardwareMap.servo["clawServo"]
        val intakeServo = hardwareMap.crservo["intakeServo"]
        val clawRotateServo = hardwareMap.servo["clawRotate"]
        val hangPusher = hardwareMap.servo["hangPusher"]

        val specimenUp = 2000
        val specimenDown = 1900
        val highBasket = 3000
        val clawOpen = 0.5
        val clawClose = 0.1
        val extendReach = 2000

        drive.poseEstimate = Pose2d(-34.09, -63.19, Math.toRadians(-90.00))
        // Setup up the trajectory sequence (drive path)

        //STILL NEED TO CONNECT EVERYTHING TOGETHER
        val traj1: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-34.09, -63.19, Math.toRadians(-90.00)))
                .setReversed(true)
                .lineToConstantHeading(Vector2d(-10.04, -34.01))
                .build()

        val traj2: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-10.4,-34.01, Math.toRadians(-90.0)))
                .setReversed(false)
                .splineTo(Vector2d(-47.87, -39.89), Math.toRadians(90.00))
                .build()

        val traj3: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-47.87,-39.89, Math.toRadians(90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .build()

        val traj4: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .back(2.0)
                .build()

        val traj5: TrajectorySequence? =
            drive.trajectorySequenceBuilder(Pose2d(-57.26, -57.26, Math.toRadians(225.00)))
                .setReversed(false)
                .lineToLinearHeading(Pose2d(-57.75, -38.37, Math.toRadians(90.0)))
                .build()


        val traj6: TrajectorySequence?=
            drive.trajectorySequenceBuilder(Pose2d(-57.75,-38.37, Math.toRadians(90.0)))
                .setReversed(true)
                .lineToLinearHeading(Pose2d(-55.0, -55.0, Math.toRadians(45.00)))
                .build()

        val traj7: TrajectorySequence?=
            drive.trajectorySequenceBuilder(Pose2d(-55.0,-55.0, Math.toRadians(45.0)))
                .back(2.0)
                .setReversed(false)
                .build()


        // Tell the User the Robot has been initialized
        telemetry.addData("Status", "Initialized")
        telemetry.update()


        // Wait for the game to start (driver presses PLAY)
        waitForStart()
        telemetry.addLine("started")
        telemetry.update()
        if (isStopRequested) return
        telemetry.addLine("not stopped")
        telemetry.update()


        //PLACE SPECIMEN ON HIGH BAR
        slideVerticalMotor.targetPosition = specimenUp
        slideVerticalMotor.power = 1.0

        drive.followTrajectorySequence(traj1)

        slideVerticalMotor.targetPosition = specimenDown
        slideVerticalMotor.power = 1.0
        sleep(200)
        clawServo.position = clawOpen
        sleep(200)
        slideVerticalMotor.targetPosition = specimenUp+50
        slideVerticalMotor.power = 1.0
        sleep(100)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 0.5

        //GO TO AND PICK UP FIRST SAMPLE
        drive.followTrajectorySequence(traj2)

        slideVerticalMotor.targetPosition = extendReach
        slideVerticalMotor.power = 1.0
        sleep(1500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 1.0
        //TransferSystemGoes Here

        //MOVE TO HIGH BASKET AND PLACE SAMPLE
        drive.followTrajectorySequence(traj3)

        slideVerticalMotor.targetPosition = highBasket
        slideVerticalMotor.power = 1.0
        sleep(1000)

        drive.followTrajectorySequence(traj4)

        clawServo.position = clawOpen
        sleep(100)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 0.5
        sleep(500)


        //MOVE TO 2nd PIXEL AND PICK UP
        drive.followTrajectorySequence(traj5)

        slideVerticalMotor.targetPosition = extendReach
        slideVerticalMotor.power = 1.0
        sleep(1500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 1.0
        //TransferSystemGoes Here

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj6)

        slideVerticalMotor.targetPosition = highBasket
        slideVerticalMotor.power = 1.0
        sleep(1000)

        drive.followTrajectorySequence(traj7)

        clawServo.position = clawOpen
        sleep(100)
        slideVerticalMotor.targetPosition = 0
        slideVerticalMotor.power = 0.5
        sleep(500)

        while (opModeIsActive() && !isStopRequested) {
            drive.update()
        }
        telemetry.addLine("tried to run code")
        telemetry.update()
        sleep(1000)
        val (x, y, heading) = drive.poseEstimate
        telemetry.addData(
            "Current Position",
            String.format(Locale.ENGLISH, "X: %f, Y: %f, and Rotation: %f", x, y, heading)
        )
        telemetry.update()
        PoseStorage.currentPose = drive.poseEstimate
    }
}