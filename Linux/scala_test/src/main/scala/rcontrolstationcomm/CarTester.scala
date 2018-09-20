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

// Var so that they can be changed from the interactive console.
object TestSettings {
  var carId = 0
  var carRoute = 3
  var startRoute = 0
  var recoveryRoute = 1
  var outerRoute = 2
  var cutouts = List(4, 5, 6, 7)
  var minKmh = 3.0
  var maxKmh = 12.0
  var aheadMargin = 20
  var aheadMarginRecovery = 15
  var recoveryGenAttampts = 100
  var fiActive = true
  var uwbMaxDiff = 1.0
  var shorenRecoveryRoute = true
}

object TestResult {
  val commands = Buffer.empty[CarSpec.Command]
  val uwbDiff = Buffer.empty[Double]
  val maxSpeed = Buffer.empty[Double]
}

class Car {
  var recoveryOk = true
  var routeInfo = new RouteInfo
  var lastSegments = List.empty[RpPoint]
  
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
  
  def stopCar(): Unit = {
    if (routeInfo.hasRoute() && !lastSegments.isEmpty) {
      println("[CarCmd] Running slowdown route")
      val r = routeInfo.generateRouteWithin(5, lastSegments.asJava, 0.8, TestSettings.aheadMargin)
      addRoute(TestSettings.carId, r.subList(lastSegments.size, r.size), false, false, TestSettings.carRoute, 2000)
      waitUntilRouteAlmostEnded(TestSettings.carId, 3)
    }
    
    println("[CarCmd] Stopping car")
    brakeAndWaitUntilStoppedPolling(TestSettings.carId, 50.0)
  }
  
  def waitCarPolling(ms: Int): Unit = {
    println("[CarCmd] Waiting for " + ms + " ms while polling the car position")
    waitPolling(TestSettings.carId, ms)
  }

  def runSegments(route: List[RpPoint]): Boolean = {
    println("[CarCmd] Running segments")
    if (recoveryOk) {
      lastSegments = route
      addRoute(TestSettings.carId, route.asJava, false, false, TestSettings.carRoute, 2000)
      val res = new WaitRouteResult
      waitUntilRouteAlmostEnded(TestSettings.carId, 4, res)
      TestResult.uwbDiff += res.maxUwbDiff
      TestResult.maxSpeed += res.maxSpeed
      if (res.maxUwbDiff > TestSettings.uwbMaxDiff) {
        println("[Error] Too large difference between the UWB-based and RTKGNSS-based positions.");
        false
      } else {
        true
      }
    } else {
      println("[ERROR] Cannot run segments because following the recovery route failed")
      false
    }
  }
  
  def runRecoveryRoute(): Boolean = {
    println("[CarCmd] Following recovery route")
    followRecoveryRoute(TestSettings.carId, TestSettings.recoveryRoute)
    recoveryOk = true
    true
  }
  
  def runRecoveryRouteV2(ri: RouteInfo, carRoute: Int): Boolean = {
    println("[CarCmd] Following recovery route (V2)")
    recoveryOk = followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, ri,
        carRoute, TestSettings.aheadMarginRecovery, TestSettings.recoveryGenAttampts,
        TestSettings.shorenRecoveryRoute)
    recoveryOk
  }

  def addFault(probe: String, faultType: String,
               param: Double, start: Int, duration: Int): Boolean = {
    println("[CarCmd] Adding fault; Probe: " + probe + " Type: " + faultType +
      " Param: " + param + " Start: " + start + " Duration: " + duration)
    fiSetEnabled(TestSettings.carId, true, 1000)
    if (TestSettings.fiActive) {
      fiAddFault(TestSettings.carId, probe, faultType, param, start, duration, 1000)
    } else {
      true
    }
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
    route:      List[RpPoint],
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
      getRoute(0, TestSettings.startRoute, 1000).asScala.toList,
      new RouteInfo(getRoute(0, TestSettings.outerRoute, 1000)),
      0)
    for (ind <- TestSettings.cutouts) s.routeInfo.addCutout(getRoute(0, ind, 1000))
    s
  }

  def newSut(state: State): Sut = {
    println("New SUT created")
    val sut = new Sut
    sut.routeInfo = state.routeInfo
    sut.clearFaults()
    sut.clearRoute()
    sut.runRecoveryRouteV2(state.routeInfo, TestSettings.carRoute)
    sut.clearRoute()
    sut.resetUwbPosNow()
    sut.activateAutopilot(true)
    sut.runSegments(state.route)
    TestResult.commands.clear()
    TestResult.uwbDiff.clear()
    TestResult.maxSpeed.clear()
    sut
  }

  def destroySut(sut: Sut): Unit = {
    sut.stopCar()
  }

  def genCommand(state: State): Gen[Command] = Gen.frequency(
    (4, genRunSegment(state)),
    (0, genFaultTest(state)),
    (2, genFaultAnchor(state)),
    (1, genFaultWheelSlip(state)),
    (1, genFaultYaw(state)))

  def genRunSegment(state: State): Gen[RunSegment] = for {
    seed <- Gen.choose(-12000, 12000)
    points <- Gen.choose(4, 7)
    speed <- Gen.choose(TestSettings.minKmh / 3.6 * 10.0, TestSettings.maxKmh / 3.6 * 10.0)
  } yield {
    state.routeInfo.setRandomSeed(seed)
    val r = state.routeInfo.generateRouteWithin(
      points, state.route.asJava, speed.toDouble / 10.0, TestSettings.aheadMargin)
    println("Segment size: " + r.size)
    RunSegment(r.subList(state.route.size, r.size).asScala.toList)
  }
  
  def genFaultTest(state: State): Gen[AddFault] = for {
    probe <- Gen.oneOf("px", "px")
    faultType <- Gen.oneOf("OFFSET", "AMPLIFICATION")
    param <- Gen.choose(0, 7)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 10)
  } yield AddFault(probe, faultType, param, start, duration)

  def genFaultAnchor(state: State): Gen[AddFault] = for {
    probe <- Gen.oneOf("uwb_range_50", "uwb_range_234")
    faultType <- Gen.oneOf("OFFSET", "AMPLIFICATION")
    param <- Gen.choose(0, 5)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 8)
  } yield AddFault(probe, faultType, param, start, duration)
  
  def genFaultWheelSlip(state: State): Gen[AddFault] = for {
    param <- Gen.choose(10, 50)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 10)
  } yield AddFault("uwb_travel_dist", "AMPLIFICATION", param.toDouble / 10.0, start, duration)
  
  def genFaultYaw(state: State): Gen[AddFault] = for {
    param <- Gen.choose(-20, 20)
    start <- Gen.choose(0, 100)
    duration <- Gen.choose(1, 10)
  } yield AddFault("uwb_yaw", "OFFSET", param, start, duration)

  case class RunSegment(route: List[RpPoint]) extends Command {
    type Result = Boolean

    def run(sut: Sut): Result = {
      TestResult.commands += this
      sut.runSegments(route)
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
      TestResult.commands += this
      sut.addFault(probe, fault, param, start, duration)
    }

    def nextState(state: State): State = {
      state.copy(faultNum = state.faultNum + 1)
    }

    def preCondition(state: State): Boolean = true

    // This command should always succeed (never throw an exception)
    def postCondition(state: State, result: Try[Result]) =
      result == Success(true)
  }
}

object CarTester {
  def main(args: Array[String]): Unit = {
    connect("localhost", 65191)

    //randomDrivingTest()
    //    randomGenTest()

    testScala(3)
    
    //    runLastTest()
    //    runLastTestHdd()

    disconnect()
  }

  def connect(host: String, port: Int): Boolean =
    rcsc_connectTcp(pointerToCString(host), port)

  def disconnect(): Unit = rcsc_disconnectTcp()
  
  def clearBuffers(): Unit = rcsc_clearBuffers()

  def testScala(tests: Int) {
    val params = Test.Parameters.default.
      withMinSuccessfulTests(tests).
      withMaxSize(20).
      withMinSize(5).
      withMaxDiscardRatio(10)

    // Interactive way
//    CarSpec.propertyNoShrink().check(params)

    // NOTE: If a command cannot be generated because the precondition always fails
    // the entire test will fail.
    val res = Test.check(params, CarSpec.propertyNoShrink())

    println("Tests passed: " + res.succeeded)
    println("Time: " + res.time + " ms")

    if (res.passed) {
      println("All tests passed")
    } else {
      println("Tests failed. Failing command sequence:")
      for (cmd <- TestResult.commands) println(cmd.toString())
    }
    
    println("test_diff = [")
    for (diff <- TestResult.uwbDiff) println(diff)
    println("]';")
    
    println("test_speed = [")
    for (speed <- TestResult.maxSpeed) println(speed)
    println("]';")

    // Save last test to disk to make re-running it possible
    val oos = new ObjectOutputStream(new FileOutputStream("last_test.bin"))
    oos.writeObject(TestResult.commands.toList)
    oos.close
    
    val oos2 = new ObjectOutputStream(new FileOutputStream("last_test_uwb_diff.bin"))
    oos2.writeObject(TestResult.uwbDiff.toList)
    oos2.close
    
    val oos3 = new ObjectOutputStream(new FileOutputStream("last_test_speed.bin"))
    oos3.writeObject(TestResult.maxSpeed.toList)
    oos3.close
  }

  def runLastTest() = rerunTest(TestResult.commands.clone())

  def runLastTestHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test.bin"))
    val cmds = ois.readObject.asInstanceOf[List[CarSpec.Command]].toBuffer
    ois.close
    rerunTest(cmds)
  }
  
  def printLastTest() {
    for (cmd <- TestResult.commands) println(cmd)
  }
  
  def printLastTestUwbDiff() {
    println("test_diff = [")
    for (diff <- TestResult.uwbDiff) println(diff)
    println("]';")
  }
  
  def printLastTestMaxSpeed() {
    println("test_speed = [")
    for (speed <- TestResult.maxSpeed) println(speed * 3.6)
    println("]';")
  }
  
  def printLastTestHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test.bin"))
    val cmds = ois.readObject.asInstanceOf[List[CarSpec.Command]].toBuffer
    ois.close
    for (cmd <- cmds) println(cmd)
  }
  
  def printLastTestUwbDiffHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test_uwb_diff.bin"))
    val diffs = ois.readObject.asInstanceOf[List[Double]].toBuffer
    ois.close
    println("test_diff = [")
    for (diff <- diffs) println(diff)
    println("]';")
  }
  
  def printLastTestMaxSpeedHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test_speed.bin"))
    val speeds = ois.readObject.asInstanceOf[List[Double]].toBuffer
    ois.close
    println("test_speed = [")
    for (speed <- speeds) println(speed * 3.6)
    println("]';")
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
    
    CarSpec.destroySut(sut);

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
      followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, r,
          TestSettings.carRoute, TestSettings.aheadMarginRecovery,
          TestSettings.recoveryGenAttampts, TestSettings.shorenRecoveryRoute)
      rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 5000)
      rcsc_setAutopilotActive(TestSettings.carId, true, 2000)

      for (i <- 0 to 4) {
        rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), TestSettings.aheadMargin)

        val subRoute = rGen.subList(indLast, rGen.size())
        if (subRoute.size() > 0) {
          addRoute(TestSettings.carId, subRoute, false, false, TestSettings.carRoute, 2000)
        }
        waitUntilRouteAlmostEnded(TestSettings.carId, 4)
        indLast = rGen.size()
      }
      brakeAndWaitUntilStoppedPolling(TestSettings.carId, 50.0)
    }
  }
  
  def recoveryTest() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))
    rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 5000)
    followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, r,
        TestSettings.carRoute, TestSettings.aheadMarginRecovery,
        TestSettings.recoveryGenAttampts, TestSettings.shorenRecoveryRoute)
    brakeAndWaitUntilStoppedPolling(TestSettings.carId, 50.0)
  }
  
  def recoveryTestGenOnly() {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))
    r.setRandomSeed(123) // For repeatedly comparing the algorithm
    rcsc_clearRoute(TestSettings.carId, TestSettings.carRoute, 5000)
    followRecoveryRouteV2(TestSettings.carId, TestSettings.recoveryRoute, r,
        TestSettings.carRoute, TestSettings.aheadMarginRecovery, TestSettings.recoveryGenAttampts,
        TestSettings.shorenRecoveryRoute, true)
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
        rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), TestSettings.aheadMargin)

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
        Thread.sleep(5)
        indLast = rGen.size()
      }
    }

    println("Maximum outer loop attempts during this run: " + maxAttempts)
    println("Total amount of generated points: " + genPoints)
    println("Used points: " + usedPoints)
    println("Point usage average: " +
      (usedPoints.toDouble / genPoints.toDouble) * 100.0 + " %")
  }
  
  def longGenTest(points: Int) {
    val edgeRoute = getRoute(0, TestSettings.outerRoute, 5000);
    val startRoute = getRoute(0, TestSettings.startRoute, 5000);
    var maxAttempts = 0
    var genPoints = 0
    var usedPoints = 0
    val r = new RouteInfo(edgeRoute);
    for (ind <- TestSettings.cutouts) r.addCutout(getRoute(0, ind, 1000))

    var rGen = startRoute
    var indLast = 0

    rcsc_clearRoute(-1, 3, 5000)

    for (i <- 0 to (points / 5)) {
      rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), TestSettings.aheadMargin)

      if (r.getLastOuterAttempts() > maxAttempts) {
        maxAttempts = r.getLastOuterAttempts()
      }

      genPoints += r.getLastGeneratedPoints()
      usedPoints += 5

      addRoute(0, rGen, true, true, TestSettings.carRoute, 2000)
      
      Thread.sleep(5)
      indLast = rGen.size()
    }

    println("Maximum outer loop attempts during this run: " + maxAttempts)
    println("Total amount of generated points: " + genPoints)
    println("Used points: " + usedPoints)
    println("Point usage average: " +
      (usedPoints.toDouble / genPoints.toDouble) * 100.0 + " %")
  }

  def driveRoute(route: Int) {
    val r = getRoute(TestSettings.carId, route, 1000)
    if (!r.isEmpty()) {
      addRoute(TestSettings.carId, r, true, false, -2, 5000)
      rcsc_setAutopilotActive(TestSettings.carId, true, 2000)
      waitUntilRouteAlmostEnded(TestSettings.carId, 2)
      brakeAndWaitUntilStoppedPolling(TestSettings.carId, 50.0)
    }
  }

  def driveCarRoute(): Unit = driveRoute(TestSettings.carRoute)

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
    val a = List.fill(3)(new RpPoint)
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
