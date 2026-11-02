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
import com.seattlesolvers.solverslib.command.InstantCommand;
import com.seattlesolvers.solverslib.command.ParallelRaceGroup;
import com.seattlesolvers.solverslib.command.RunCommand;
import com.seattlesolvers.solverslib.command.SequentialCommandGroup;
import com.seattlesolvers.solverslib.command.WaitCommand;
import com.seattlesolvers.solverslib.command.WaitUntilCommand;
import com.seattlesolvers.solverslib.pedroCommand.FollowPathCommand;
import com.turtletracerlib.TurtleTracerReader;
import com.turtletracerlib.auto.Alliance;
import com.turtletracerlib.auto.AutoContext;
import com.turtletracerlib.auto.AutoSelection;
import com.turtletracerlib.auto.MirrorPolicy;
import com.turtletracerlib.auto.TurtleAuto;
import com.turtletracerlib.pathing.NamedCommands;
import com.turtletracerlib.pathing.ProgressTracker;
import java.io.IOException;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@TurtleAuto(
  name = "AutoPath",
  drawnFor = Alliance.RED,
  mirror = MirrorPolicy.FOLLOW_ALLIANCE,
  overrideMirror = true
)
public class AutoPath extends SequentialCommandGroup {

  private final Follower follower;
  private final AutoContext ctx; // The driver's alliance, for paths drawn for RED
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

  public AutoPath(
    final Follower follower,
    HardwareMap hw,
    Telemetry telemetry,
    AutoContext autoContext
  ) throws IOException {
    this.follower = follower;
    // Drawn for the red alliance, and mirrored when the driver picks the other one.
    this.ctx = autoContext
      .drawnFor(Alliance.RED)
      .mirroring(MirrorPolicy.FOLLOW_ALLIANCE);
    AutoSelection.record(ctx); // The alliance it runs for, and whether it's mirrored, for TeleOp

    pp = new TurtleTracerReader("AutoPath.turt", hw.appContext).mirrored(
      ctx.mirror()
    );
    pp.onEvent("SpinUp", NamedCommands.getCommand("SpinUp")).onEvent(
      "Intake",
      NamedCommands.getCommand("Intake")
    );

    tracker = new ProgressTracker(follower, telemetry);

    // Load poses
    startPoint = pp.get("startPoint");
    DriveToShoot = pp.get("DriveToShoot");
    DriveToShoot_line0_control1 = pp.get("DriveToShoot_line0_control1");
    CollectRow1 = pp.get("CollectRow1");
    CollectRow1_line1_control1 = pp.get("CollectRow1_line1_control1");
    ReturnToShoot = pp.get("ReturnToShoot");

    follower.setPose(startPoint);

    buildPaths();

    addCommands(
      new InstantCommand(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 0, 0);
        tracker.setCurrentPath(startPointTODriveToShoot);
      }),
      new ParallelRaceGroup(
        new FollowPathCommand(follower, startPointTODriveToShoot),
        new RunCommand(() -> tracker.update())
      ),
      new WaitCommand(1500),
      new FollowPathCommand(follower, DriveToShootTOCollectRow1),
      new InstantCommand(() -> {
        tracker.clearPathEvents();
        pp.registerLineEvents(tracker, 2, 0);
        tracker.setCurrentPath(CollectRow1TOReturnToShoot);
      }),
      new ParallelRaceGroup(
        new FollowPathCommand(follower, CollectRow1TOReturnToShoot),
        new RunCommand(() -> tracker.update())
      )
    );
  }

  public void buildPaths() {
    startPointTODriveToShoot = curve(
      startPoint,
      DriveToShoot_line0_control1,
      DriveToShoot
    ).linear(ctx.heading(Math.toRadians(0)), ctx.heading(Math.toRadians(45)));

    DriveToShootTOCollectRow1 = curve(
      DriveToShoot,
      CollectRow1_line1_control1,
      CollectRow1
    ).tangent();

    CollectRow1TOReturnToShoot = line(CollectRow1, ReturnToShoot).linear(
      ctx.heading(Math.toRadians(90)),
      ctx.heading(Math.toRadians(45))
    );
  }
}
