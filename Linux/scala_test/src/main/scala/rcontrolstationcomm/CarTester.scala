/*
    Copyright 2018 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package rcontrolstationcomm

import RControlStationCommLibrary._
import CAR_STATE._
import org.bridj.Pointer._
import Utils._
import scala.collection.JavaConverters._
import scala.collection.mutable.Buffer
import scala.collection.mutable.ArrayBuffer
import java.io._

import org.scalacheck.Gen
import org.scalacheck.Prop
import org.scalacheck.Test
import util.{ Try, Success, Failure }

object TestSettings {
  val carId = 1
  val carRoute = 3
  val startRoute = 0
  val recoveryRoute = 1
  val outerRoute = 2
  val cutouts = List(4, 5, 6, 7)
}

object TestData {
  val commands = Buffer.empty[CarSpec.Command]
}

@SerialVersionUID(1L)
class Rp(p: ROUTE_POINT) extends Serializable {
  val px = p.px()
  val py = p.py()
  val speed = p.speed()
  val time = p.time()
  def toRoutePoint(): ROUTE_POINT = {
    val rp = new ROUTE_POINT()
    rp.px(px)
    rp.py(py)
    rp.speed(speed)
    rp.time(time)
    rp
  }
}

@SerialVersionUID(1L)
class RpList(r: List[Rp]) extends Serializable {
  val route = r
  def this(r: Buffer[ROUTE_POINT]) { this(r.map(new Rp(_)).toList) }
  def toRoutePoints(): List[ROUTE_POINT] = route.map(_.toRoutePoint())
  def size(): Int = route.size
  def ++(other: RpList): RpList = new RpList(route ++ other.route)
}

class Car {
  def clearRoute(): Boolean = {
    println("[CarCmd] Clearing route")
    rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 1000)
  }
  
  def activateAutopilot(active: Boolean): Boolean = {
    if (active) {
      println("[CarCmd] Activating autopilot")
    } else {
      println("[CarCmd] Deactivating autopilot")
    }
    rcsc_setAutopilotActive(TestSettings.carId, active, 2000)
  }
  
  def brakeCar(timeMs: Int): Unit = {
    rcsc_rcControl(TestSettings.carId, 3, 50.0, 0.0);
    waitPolling(TestSettings.carId, timeMs)
  }
  
  def waitCarPolling(ms: Int): Unit = {
    println("[CarCmd] Waiting for " + ms + " ms while polling the car position")
    waitPolling(TestSettings.carId, ms)
  }
  
  def runSegments(route: List[ROUTE_POINT]): Boolean = {
    println("[CarCmd] Running generated segments")
    addRoute(TestSettings.carId, route.asJava, false, false, TestSettings.carRoute, 2000)
    waitUntilRouteAlmostEnded(TestSettings.carId, 4)
  }
  
  def runRecoveryRoute(): Boolean = {
    println("[CarCmd] Following recovery route")
    followRecoveryRoute(TestSettings.carId, TestSettings.recoveryRoute)
    true
  }

  def addFault(probe: String, faultType: String,
               param: Double, start: Int, duration: Int): Boolean = {
    println("[CarCmd] Adding fault; Probe: " + probe + " Type: " + faultType +
      " Param: " + param + " Start: " + start + " Duration: " + duration)
    fiSetEnabled(TestSettings.carId, true, 1000)
    fiAddFault(TestSettings.carId, probe, faultType, param, start, duration, 1000)
  }

  def clearFaults(): Boolean = {
    println("[CarCmd] Clearing faults")
    fiClearFaults(TestSettings.carId, 1000)
  }
  
  def resetUwbPosNow(): Boolean = {
    println("[CarCmd] Resetting UWB position")
    resetUwbPos(TestSettings.carId, 1000)
  }
}

object CarSpec extends Commands2 {
  case class State(
    route:      RpList,
    startRoute: RpList,
    routeInfo:  RouteInfo,
    faultNum:   Int)

  type Sut = Car

  def canCreateNewSut(newState: State, initSuts: Traversable[State],
                      runningSuts: Traversable[Sut]): Boolean = {
    initSuts.isEmpty && runningSuts.isEmpty
  }

  def initialPreCondition(state: State): Boolean = true

  def genInitialState: Gen[State] = {
    println("New initial state created")
    val s = State(
      new RpList(Buffer.empty[ROUTE_POINT]),
      new RpList(getRoute(0, TestSettings.startRoute, 1000).asScala),
      new RouteInfo(getRoute(0, TestSettings.outerRoute, 1000)),
      0)
    for (ind <- TestSettings.cutouts) s.routeInfo.addCutout(getRoute(0, ind, 1000))
    s
  }

  def newSut(state: State): Sut = {
    println("New SUT created")
    val sut = new Sut
    sut.clearFaults()
    sut.clearRoute()
    sut.runRecoveryRoute()
    sut.resetUwbPosNow()
    sut.activateAutopilot(true)
    TestData.commands.clear()
    sut
  }

  def destroySut(sut: Sut): Unit = {
    sut.activateAutopilot(false)
    sut.brakeCar(2000)
  }

  def genCommand(state: State): Gen[Command] = Gen.frequency(
    (3, genRunSegment(state)),
    (2, genFaultAnchor(state)),
    (0, genFaultTest(state)))

  def genRunSegment(state: State): Gen[RunSegment] = for {
    seed <- Gen.choose(-12000, 12000)
    points <- Gen.choose(4, 7)
    speed <- Gen.choose(9, 15)
  } yield {
    state.routeInfo.setRandomSeed(seed)

    val r = state.routeInfo.generateRouteWithin(
      points,
      (state.route.size match {
        case 0 => state.startRoute.toRoutePoints()
        case _ => state.route.toRoutePoints()
      }).asJava, speed.toDouble / 10.0, 20)

    println("Segment size: " + r.size)

    RunSegment(new RpList(r.subList(state.route.size, r.size).asScala))
  }

  def genFaultAnchor(state: State): Gen[AddFault] = for {
    probe <- Gen.oneOf("uwb_range_50", "uwb_range_234")
    faultType <- Gen.oneOf("OFFSET", "AMPLIFICATION")
    param <- Gen.choose(0, 5)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 8)
  } yield AddFault(probe, faultType, param, start, duration)

  def genFaultTest(state: State): Gen[AddFault] = for {
    probe <- Gen.oneOf("px", "px")
    faultType <- Gen.oneOf("OFFSET", "AMPLIFICATION")
    param <- Gen.choose(0, 7)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 10)
  } yield AddFault(probe, faultType, param, start, duration)

  case class RunSegment(route: RpList) extends Command {
    type Result = Boolean

    def run(sut: Sut): Result = {
      TestData.commands += this
      sut.runSegments(route.toRoutePoints().toList)
    }

    def nextState(state: State): State = {
      state.copy(route = state.route ++ route)
    }

    // This command has no preconditions
    def preCondition(state: State): Boolean = true

    // This command should always succeed (never throw an exception)
    def postCondition(state: State, result: Try[Result]) =
      result == Success(true)
  }

  case class AddFault(probe: String, fault: String, param: Double,
                      start: Int, duration: Int) extends Command {
    type Result = Boolean

    def run(sut: Sut): Result = {
      TestData.commands += this
      sut.addFault(probe, fault, param, start, duration)
    }

    def nextState(state: State): State = {
      state.copy(faultNum = state.faultNum + 1)
    }

    def preCondition(state: State): Boolean = state.faultNum < 4

    // This command should always succeed (never throw an exception)
    def postCondition(state: State, result: Try[Result]) =
      result == Success(true)
  }
}

object CarTester {
  def main(args: Array[String]): Unit = {
    connect("localhost", 65191)

    randomDrivingTest()
    //    randomGenTest()

//    testScala(3)
    
    //    runLastTest()
    //    runLastTestHdd()

    disconnect()
  }

  def connect(host: String, port: Int): Boolean =
    rcsc_connectTcp(pointerToCString(host), port)

  def disconnect(): Unit =
    rcsc_disconnectTcp()

  def testScala(tests: Int) {
    val params = Test.Parameters.default.
      withMinSuccessfulTests(tests).
      withMaxSize(20).
      withMinSize(5)

    // Interactive way
//    CarSpec.propertyNoShrink().check(params)

    val res = Test.check(params, CarSpec.propertyNoShrink())
    
    println("Tests passed: " + res.succeeded)
    println("Time: " + res.time + " ms")
    
    if (res.passed) {
      println("All tests passed")
    } else {
      println("Tests failed. Failing command sequence:")
      for (cmd <- TestData.commands) println(cmd.toString())
    }

    // Save last test to disk to make re-running it possible
    val oos = new ObjectOutputStream(new FileOutputStream("last_test.bin"))
    oos.writeObject(TestData.commands.toList)
    oos.close
  }

  def runLastTest() = rerunTest(TestData.commands.clone())

  def runLastTestHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test.bin"))
    val cmds = ois.readObject.asInstanceOf[List[CarSpec.Command]].toBuffer
    ois.close
    rerunTest(cmds)
  }

  def rerunTest(cmds: Buffer[CarSpec.Command]) {
    println("Re-running test...")
    
    println("Commands to re-run")
    for (cmd <- cmds) println(cmd.toString())
    println("")

    var state = CarSpec.genInitialState.sample.get
    val sut = CarSpec.newSut(state)
    var ok = true

    for (cmd <- cmds) {
      val res = cmd.run(sut)
      state = cmd.nextState(state)
      if (cmd.postCondition(state, Try(res)) != Prop(true)) {
        ok = false
        println("Postcondition failed")
      }
    }

    if (ok) {
      println("Test sequence successful")
    } else {
      println("Test sequence failed")
    }
  }

  def randomDrivingTest() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val startRoute = getRoute(0, TestSettings.startRoute, 5000);
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))

    for (i <- 0 to 2) {
      var rGen = startRoute
      var indLast = 0

      rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 5000)
      followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, r, TestSettings.carRoute)
      rcsc_setAutopilotActive(TestSettings.carId, true, 2000)

      for (i <- 0 to 4) {
        rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), 25)

        val subRoute = rGen.subList(indLast, rGen.size())
        if (subRoute.size() > 0) {
          addRoute(TestSettings.carId, subRoute, false, false, TestSettings.carRoute, 2000)
        }
        waitUntilRouteAlmostEnded(TestSettings.carId, 4)
        indLast = rGen.size()
      }
      rcsc_setAutopilotActive(TestSettings.carId, false, 2000)
      rcsc_rcControl(TestSettings.carId, 3, 50.0, 0.0);
      waitPolling(TestSettings.carId, 2000)
    }
  }
  
  def recoveryTest() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))
    rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 5000)
    followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, r, TestSettings.carRoute)
    rcsc_setAutopilotActive(TestSettings.carId, false, 2000)
    rcsc_rcControl(TestSettings.carId, 3, 50.0, 0.0);
    waitPolling(TestSettings.carId, 2000)
  }

  def randomGenTest() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val startRoute = getRoute(0, TestSettings.startRoute, 5000);
    var maxAttempts = 0
    var genPoints = 0
    var usedPoints = 0
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))

    for (i <- 0 to 5) {
      var rGen = startRoute
      var indLast = 0

      rcsc_clearRoute(-1, 3, 5000)

      for (i <- 0 to 200) {
        rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), 25)

        if (r.getLastOuterAttempts() > maxAttempts) {
          maxAttempts = r.getLastOuterAttempts()
        }

        genPoints += r.getLastGeneratedPoints()
        usedPoints += 5

        var start = indLast - 20
        if (start < 0) {
          start = 0
        }

        val subRoute = rGen.subList(start, rGen.size())
        if (subRoute.size() > 0) {
          addRoute(0, subRoute, true, true, TestSettings.carRoute, 2000)
        }
        Thread.sleep(40)
        indLast = rGen.size()
      }
    }

    println("Maximum outer loop attempts during this run: " + maxAttempts)
    println("Total amount of generated points: " + genPoints)
    println("Used points: " + usedPoints)
    println("Point usage average: " +
      (usedPoints.toDouble / genPoints.toDouble) * 100.0 + " %")
  }

  def getPrintRouteTest() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    for (r <- edgeRoute.asScala) {
      println("[" + r.px() + ", " + r.py() + "]")
    }

    val r = new RouteInfo(edgeRoute);
    println("XMin: " + r.xMin())
    println("XMax: " + r.xMax())
    println("YMin: " + r.yMin())
    println("YMax: " + r.yMax())
    println("Len: " + r.length())
  }

  def carStateTest() {
    val st = getCarState(TestSettings.carId, 5000)
    println("px: " + st.px() + ", py: " + st.py() + ", yaw: " + st.yaw())
    println("ap_route_left: " + st.ap_route_left())
  }

  def createManTest() {
    val a = List.fill(3)(new ROUTE_POINT)
    a(0).px(5)
    a(0).py(8)
    a(0).speed(3.1)
    a(0).time(2900)

    a(1).px(2)
    a(1).py(4)
    a(1).speed(3.1)
    a(1).time(2920)

    a(2).px(-2)
    a(2).py(4)
    a(2).speed(2.1)
    a(2).time(2980)

    addRoute(0, a.asJava, true, true, TestSettings.carRoute, 5000)
  }
}
