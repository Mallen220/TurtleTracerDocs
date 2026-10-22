package org.firstinspires.ftc.teamcode.Commands.AutoCommands;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.api.Paths.path;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.turtletracerlib.TurtleTracerReader;
import com.turtletracerlib.pathing.NamedCommands;
import com.turtletracerlib.pathing.ProgressTracker;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.delays.WaitUntil;
import dev.nextftc.core.commands.groups.ParallelRaceGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import java.io.IOException;

public class AutoPath extends Command {

  private final Follower follower;
  private final PoseFactory p = PoseFactory.degrees();
  private TurtleTracerReader pp;
  private ProgressTracker tracker;
  private Command group;

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

  @Override
  public void start() {
    buildPaths();
    group = new SequentialGroup(
      new InstantCommand(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 0, 0);
        tracker.setCurrentPath(startPointTODriveToShoot);
      }),
      new ParallelRaceGroup(
        new LambdaCommand("FollowPath")
          .setStart(() -> follower.follow(startPointTODriveToShoot))
          .setIsDone(() -> !follower.isBusy()),
        new LambdaCommand("Update")
          .setUpdate(() -> tracker.update())
          .setIsDone(() -> false)
      ),
      new Delay(1.500),
      new LambdaCommand("FollowPath")
        .setStart(() -> follower.follow(DriveToShootTOCollectRow1))
        .setIsDone(() -> !follower.isBusy()),
      new InstantCommand(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 2, 0);
        tracker.setCurrentPath(CollectRow1TOReturnToShoot);
      }),
      new ParallelRaceGroup(
        new LambdaCommand("FollowPath")
          .setStart(() -> follower.follow(CollectRow1TOReturnToShoot))
          .setIsDone(() -> !follower.isBusy()),
        new LambdaCommand("Update")
          .setUpdate(() -> tracker.update())
          .setIsDone(() -> false)
      )
    );
    group.start();
  }

  @Override
  public void update() {
    if (group != null) group.update();
  }

  @Override
  public void stop(boolean interrupted) {
    if (group != null) group.stop(interrupted);
  }

  @Override
  public boolean isDone() {
    return group != null && group.isDone();
  }
}
