package org.firstinspires.ftc.teamcode;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.turtletracerlib.pathing.NamedCommands;
import com.turtletracerlib.pathing.ProgressTracker;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "Turtle Tracer Autonomous", group = "Autonomous")
@Configurable // Panels
public class TurtleTracerAutonomous extends OpMode {

  private TelemetryManager panelsTelemetry; // Panels Telemetry instance
  public Follower follower; // Pathing follower instance
  private final PoseFactory p = PoseFactory.degrees();
  private ProgressTracker tracker; // Progress tracker instance for event markers
  private int pathState; // Current autonomous path state (state machine)
  private ElapsedTime pathTimer; // Timer for path state machine
  private Paths paths; // Paths defined in the Paths class

  @Override
  public void init() {
    panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    // ...
    panelsTelemetry.debug("Status", "Initialized");
    panelsTelemetry.update(telemetry);

    follower = Constants.create(hardwareMap);
    follower.setPose(p.of(9.000, 25.000, 0.000));

    org.firstinspires.ftc.teamcode.RobotCommands.register(hardwareMap); // Team's named commands

    pathTimer = new ElapsedTime();
    paths = new Paths(follower); // Build paths

    tracker = new ProgressTracker(follower, telemetry);
  }

  @Override
  public void loop() {
    follower.update(); // Update follower
    tracker.update(); // Update tracker
    pathState = autonomousPathUpdate(); // Update autonomous state machine

    // Log values to Panels and Driver Station
    panelsTelemetry.debug("Path State", pathState);
    panelsTelemetry.debug("X", follower.pose().x());
    panelsTelemetry.debug("Y", follower.pose().y());
    panelsTelemetry.debug("Heading", follower.pose().heading());
    panelsTelemetry.update(telemetry);
  }

  public static class Paths {

    private static final PoseFactory p = PoseFactory.degrees();
    public Path DriveToShoot;
    public Path CollectRow1;
    public Path ReturnToShoot;

    public Paths(Follower follower) {
      DriveToShoot = curve(
        p.of(9.000, 25.000, 0.0),
        p.of(28.000, 22.000, 0.0),
        p.of(48.000, 48.000, 0.0)
      ).linear(Math.toRadians(0), Math.toRadians(45));

      CollectRow1 = curve(
        p.of(48.000, 48.000, 0.0),
        p.of(30.000, 58.000, 0.0),
        p.of(34.000, 78.000, 0.0)
      ).tangent();

      ReturnToShoot = line(
        p.of(34.000, 78.000, 0.0),
        p.of(48.000, 48.000, 0.0)
      ).linear(Math.toRadians(90), Math.toRadians(45));
    }
  }

  public int autonomousPathUpdate() {
    switch (pathState) {
      case 0:
        follower.follow(paths.DriveToShoot);
        tracker.clearPathEvents();
        tracker.onParametric(0, 0.400, NamedCommands.getCommand("SpinUp"));
        tracker.setCurrentPath(paths.DriveToShoot);
        setPathState(1);
        break;
      case 1:
        if (!follower.isBusy()) {
          setPathState(2);
        }
        break;
      case 2:
        setPathState(3);
        break;
      case 3:
        if (pathTimer.milliseconds() > 1500) {
          setPathState(4);
        }
        break;
      case 4:
        follower.follow(paths.CollectRow1);
        tracker.clearPathEvents();
        tracker.setCurrentPath(paths.CollectRow1);
        setPathState(5);
        break;
      case 5:
        if (!follower.isBusy()) {
          setPathState(6);
        }
        break;
      case 6:
        follower.follow(paths.ReturnToShoot);
        tracker.clearPathEvents();
        tracker.onParametric(0, 0.500, NamedCommands.getCommand("Intake"));
        tracker.setCurrentPath(paths.ReturnToShoot);
        setPathState(7);
        break;
      case 7:
        if (!follower.isBusy()) {
          setPathState(8);
        }
        break;
      case 8:
        requestOpModeStop();
        pathState = -1;
        break;
    }
    return pathState;
  }

  public void setPathState(int pState) {
    pathState = pState;
    pathTimer.reset();
  }
}
