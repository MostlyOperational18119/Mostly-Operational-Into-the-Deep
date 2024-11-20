package org.firstinspires.ftc.teamcode.Autonomous.NoSplineAuto

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.geometry.Vector2d
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.teamcode.Autonomous.poseStorage
import org.firstinspires.ftc.teamcode.DriveMethods
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence
import java.util.Locale


// Autonomous

@Autonomous(name = "BASKET_Meet2", group = "B")
class BASKET_MEET_2 : DriveMethods() {
    override fun runOpMode() {
        // Setup Odometry :)
        val drive = SampleMecanumDrive(hardwareMap)

        val slideMotor = hardwareMap.get(DcMotor::class.java, "motorSlide")
        slideMotor.targetPosition = 0
        slideMotor.power = 0.0
        slideMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        slideMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val reachMotor = hardwareMap.get(DcMotor::class.java, "motoReach")
        reachMotor.targetPosition = 0
        reachMotor.power = 0.0
        reachMotor.mode = DcMotor.RunMode.RUN_TO_POSITION
        reachMotor.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        val clawServo = hardwareMap.get(Servo::class.java, "claw")
        val transferServo = hardwareMap.get(Servo::class.java, "rotateServo")
        val intakeServo = hardwareMap.get(CRServo::class.java, "claw")

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
        slideMotor.targetPosition = specimenUp
        slideMotor.power = 1.0

        drive.followTrajectorySequence(traj1)

        slideMotor.targetPosition = specimenDown
        slideMotor.power = 1.0
        sleep(200)
        clawServo.position = clawOpen
        sleep(200)
        slideMotor.targetPosition = specimenUp+50
        slideMotor.power = 1.0
        sleep(100)
        slideMotor.targetPosition = 0
        slideMotor.power = 0.5

        //GO TO AND PICK UP FIRST SAMPLE
        drive.followTrajectorySequence(traj2)

        reachMotor.targetPosition = extendReach
        reachMotor.power = 1.0
        sleep(1500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        reachMotor.targetPosition = 0
        reachMotor.power = 1.0
        //TransferSystemGoes Here

        //MOVE TO HIGH BASKET AND PLACE SAMPLE
        drive.followTrajectorySequence(traj3)

        slideMotor.targetPosition = highBasket
        slideMotor.power = 1.0
        sleep(1000)

        drive.followTrajectorySequence(traj4)

        clawServo.position = clawOpen
        sleep(100)
        slideMotor.targetPosition = 0
        slideMotor.power = 0.5
        sleep(500)


        //MOVE TO 2nd PIXEL AND PICK UP
        drive.followTrajectorySequence(traj5)

        reachMotor.targetPosition = extendReach
        reachMotor.power = 1.0
        sleep(1500)
        intakeServo.power = 1.0
        sleep(1000)
        intakeServo.power = 0.0
        reachMotor.targetPosition = 0
        reachMotor.power = 1.0
        //TransferSystemGoes Here

        //MOVE TO HIGH BASKET AND PLACE SECOND SAMPLE
        drive.followTrajectorySequence(traj6)

        slideMotor.targetPosition = highBasket
        slideMotor.power = 1.0
        sleep(1000)

        drive.followTrajectorySequence(traj7)

        clawServo.position = clawOpen
        sleep(100)
        slideMotor.targetPosition = 0
        slideMotor.power = 0.5
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
        poseStorage.currentPose = drive.poseEstimate
    }
}