package org.firstinspires.ftc.teamcode.Autonomous;

public class Event{
    /// If stop is true, the robot will stop when start point is reached
    /// Otherwise, the robot will continue execution of the Execution sequence.
    /// If stop is true, end pos and endCallback will not be used.
    /// If stop is false, the endCallback will run when the end pos is reached
    boolean stop;
    ///If stop at end is true, the robot will be stopped at the endpoint.
    /// The robot will only stop if the callback functions have not completed execution.
    boolean stopAtEnd;
    ///Format: ["x1", "y1"]
    int[] startPos;
    ///A function that will be called when start pos is reached. Returns nothing, takes no input
    Runnable startCallback;
    ///Format: ["x2", "y2"]
    int[] endPos;
    ///A function that will be called when end pos is reached. Returns nothing, takes no input
    Runnable endCallback;
}
