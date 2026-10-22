package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.commands.Commands;
import com.pedropathing.ivy.groups.Groups;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.turtletracerlib.TurtleTracerReader;
import com.turtletracerlib.pathing.NamedCommands;
import com.turtletracerlib.pathing.ProgressTracker;
import java.io.IOException;

/**
 * Call {@code Scheduler.reset();} in init(), start it with
 * {@code new AutoPath(follower, hardwareMap).command().schedule();}, and call
 * {@code follower.update();} and {@code Scheduler.execute();} every loop.
 */
public class AutoPath {

  private final Follower follower;
  private final PoseFactory p = PoseFactory.degrees();
  private TurtleTracerReader pp;
  private ProgressTracker tracker;

  // Poses
  private Pose startPoint;
  private Pose DriveToShoot;
  private Pose DriveToShoot_line0_control1;
  private Pose CollectRow1;
  private Pose CollectRow1_line1_control1;
  private Pose ReturnToShoot;

  // Path chains
  private Path startPointTODriveToShoot;
  private Path DriveToShootTOCollectRow1;
  private Path CollectRow1TOReturnToShoot;

  public AutoPath(final Follower follower, HardwareMap hw) throws IOException {
    this.follower = follower;

    pp = new TurtleTracerReader("AutoPath.turt", hw.appContext);
    pp.onEvent("SpinUp", NamedCommands.getCommand("SpinUp")).onEvent(
      "Intake",
      NamedCommands.getCommand("Intake")
    );

    tracker = new ProgressTracker(follower, null);

    // Load poses
    startPoint = pp.get("startPoint");
    DriveToShoot = pp.get("DriveToShoot");
    DriveToShoot_line0_control1 = pp.get("DriveToShoot_line0_control1");
    CollectRow1 = pp.get("CollectRow1");
    CollectRow1_line1_control1 = pp.get("CollectRow1_line1_control1");
    ReturnToShoot = pp.get("ReturnToShoot");

    follower.setPose(startPoint);
  }

  public void buildPaths() {
    startPointTODriveToShoot = curve(
      startPoint,
      p.of(28.000, 22.000, 0.0),
      DriveToShoot
    ).linear(Math.toRadians(0), Math.toRadians(45));

    DriveToShootTOCollectRow1 = curve(
      DriveToShoot,
      p.of(30.000, 58.000, 0.0),
      CollectRow1
    ).tangent();

    CollectRow1TOReturnToShoot = line(CollectRow1, ReturnToShoot).linear(
      Math.toRadians(90),
      Math.toRadians(45)
    );
  }

  public Command command() {
    buildPaths();
    return Groups.sequential(
      Commands.instant(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 0, 0);
        tracker.setCurrentPath(startPointTODriveToShoot);
      }),
      Groups.race(
        PedroCommands.follow(follower, startPointTODriveToShoot),
        Commands.infinite(() -> tracker.update())
      ),
      Commands.waitMs(1500),
      PedroCommands.follow(follower, DriveToShootTOCollectRow1),
      Commands.instant(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 2, 0);
        tracker.setCurrentPath(CollectRow1TOReturnToShoot);
      }),
      Groups.race(
        PedroCommands.follow(follower, CollectRow1TOReturnToShoot),
        Commands.infinite(() -> tracker.update())
      )
    );
  }
}
