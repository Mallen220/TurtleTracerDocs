package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pedro.Constants

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.TelemetryManager
import com.bylazar.telemetry.PanelsTelemetry
import com.pedropathing.api.PoseFactory
import com.pedropathing.follower.Follower
import com.pedropathing.paths.Path
import com.pedropathing.api.Paths.curve
import com.pedropathing.api.Paths.line
import com.pedropathing.api.Paths.path
import com.pedropathing.math.Pose
import com.pedropathing.paths.interpolator.Interpolator

@Autonomous(name = "Turtle Tracer Autonomous", group = "Autonomous")
@Configurable // Panels
class TurtleTracerAutonomous : OpMode() {
    private lateinit var panelsTelemetry: TelemetryManager // Panels Telemetry instance
    lateinit var follower: Follower // Pathing follower instance
    private val p = PoseFactory.degrees()
    private var pathState = 0 // Current autonomous path state (state machine)
    private lateinit var pathTimer: ElapsedTime // Timer for path state machine
    private lateinit var paths: Paths // Paths defined in the Paths class

    override fun init() {
        panelsTelemetry = PanelsTelemetry.telemetry
        // ...
        panelsTelemetry.debug("Status", "Initialized")
        panelsTelemetry.update(telemetry)

        follower = Constants.create(hardwareMap)
        follower.setPose(p.of(56.000, 8.000, 90.000))

        pathTimer = ElapsedTime()
        paths = Paths() // Build paths
    }

    override fun loop() {
        follower.update() // Update follower
        pathState = autonomousPathUpdate() // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState)
        panelsTelemetry.debug("X", follower.pose().x())
        panelsTelemetry.debug("Y", follower.pose().y())
        panelsTelemetry.debug("Heading", follower.pose().heading())
        panelsTelemetry.update(telemetry)
    }

    class Paths {
        val line1: Path

        init {
            line1 = line(
                p.of(56.000, 8.000, 0.0),
                p.of(56.000, 36.000, 0.0)
            ).linear(Math.toRadians(90.0), Math.toRadians(180.0))
        }

        companion object {
            private val p = PoseFactory.degrees()
        }
    }

    fun autonomousPathUpdate(): Int {
        when (pathState) {
            0 -> {
                follower.follow(paths.line1)
                setPathState(1)
            }
            1 -> {
                if(!follower.isBusy()) {
                    setPathState(2)
                }
            }
            2 -> {
                requestOpModeStop()
                pathState = -1
            }
        }
        return pathState
    }

    fun setPathState(pState: Int) {
        pathState = pState
        pathTimer.reset()
    }
}
