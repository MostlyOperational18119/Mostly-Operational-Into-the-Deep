package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.AbstractList;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class TrajectorySystem {
    GoBildaPinpointDriver driver;
    DcMotor motorFL, motorFR, motorBL, motorBR;
    ExecutionSequence executionSequence;
    public TrajectorySystem(GoBildaPinpointDriver driver, DcMotor motorFL, DcMotor motorFR, DcMotor motorBL, DcMotor motorBR, double xOffset, double yOffset, GoBildaPinpointDriver.GoBildaOdometryPods podType, GoBildaPinpointDriver.EncoderDirection encoderDirection1, GoBildaPinpointDriver.EncoderDirection encoderDirection2){
        this.driver = driver;
        this.motorFL = motorFL;
        this.motorFR = motorFR;
        this.motorBL = motorBL;
        this.motorBR = motorBR;
        driver.setOffsets(xOffset, yOffset);
        driver.setEncoderResolution(podType);
        driver.setEncoderDirections(
                encoderDirection1,
                encoderDirection2
        );
        driver.resetPosAndIMU();
        executionSequence = new ExecutionSequence();
        //set up complete

    }
    public void addEvent(Event event){
        executionSequence.getEventList().add(event);
    }
    public void addTrajectory(Trajectory trajectory){
        executionSequence.getTrajectoryList().add(trajectory);
    }

}

class ExecutionSequence{
    List<Event> eventList;
    List<Trajectory> trajectoryList;
    public ExecutionSequence(){
        eventList = new ArrayList<Event>();
        trajectoryList = new ArrayList<Trajectory>();
    }

    public List<Event> getEventList() {
        return eventList;
    }

    public void setEventList(List<Event> eventList) {
        this.eventList = eventList;
    }

    public List<Trajectory> getTrajectoryList() {
        return trajectoryList;
    }

    public void setTrajectoryList(List<Trajectory> trajectoryList) {
        this.trajectoryList = trajectoryList;
    }
}

