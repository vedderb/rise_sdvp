package rcontrolstationcomm

import RControlStationCommLibrary._
import CAR_STATE._
import org.bridj.Pointer._
import Utils._
import scala.collection.JavaConverters._
import scala.collection.mutable.Buffer
import scala.collection.mutable.ArrayBuffer
import java.io._

import org.scalatest.prop.Checkers
import org.scalacheck.Gen
//import org.scalacheck.commands.Commands
import org.scalacheck.Prop
import util.{Try,Success,Failure}
import org.scalactic.anyvals.{PosZInt, PosZDouble, PosInt}
import junit.framework.TestResult

object TestData {
  val commands = Buffer.empty[CarSpec.Command]
}

@SerialVersionUID(1L)
class Rp(p: ROUTE_POINT) extends Serializable {
  val px = p.px()
  val py = p.py()
  val speed = p.speed()
  val time = p.time()
  def toRoutePoint() : ROUTE_POINT = {
    val rp = new ROUTE_POINT()
    rp.px(px)
    rp.py(py)
    rp.speed(speed)
    rp.time(time)
    rp
  }
}

@SerialVersionUID(1L)
class RpList(r: Buffer[ROUTE_POINT]) extends Serializable {
  val route = r.map(new Rp(_)).toBuffer
  def toRoutePoints() = route.map(_.toRoutePoint())
  def size() = route.size
  def ++(that: RpList): RpList = {
    val newList = new RpList(Buffer.empty[ROUTE_POINT])
    newList.route.appendAll(route)
    newList.route.appendAll(that.route)
    newList
  }
}

case class CarState (
    route: RpList,
    startRoute: RpList,
    routeInfo: RouteInfo
)

class Car {
  def clearRoute() {
    rcsc_clearRoute(0, 3, 1000)
  }
  
  def runRecoveryRoute() : Boolean = {
    followRecoveryRoute(0, 1)
    true
  }
  
  def activateAutopilot(active: Boolean) {
    rcsc_setAutopilotActive(0, active, 2000)
  }
  
  def runSegments(route: List[ROUTE_POINT]) : Boolean = {
    println("runSegments")
    addRoute(0, route.asJava, false, false, 3, 2000)
    waitUntilRouteAlmostEnded(0)
    true
  }
  
  def waitCarPolling(ms: Int) {
    waitPolling(0, ms)
  }
}

object CarSpec extends Commands2 {
  type State = CarState
  type Sut = Car

  def canCreateNewSut(newState: State, initSuts: Traversable[State],
                      runningSuts: Traversable[Sut]) = {
    initSuts.isEmpty && runningSuts.isEmpty
  }
  
  def initialPreCondition(state: State): Boolean = {
    true
  }
  
  def newSut(state: State): Sut = {
    println("New SUT created")
    val sut = new Sut
    sut.clearRoute()
    sut.runRecoveryRoute()
    sut.activateAutopilot(true)
    TestData.commands.clear()
    sut
  }
  
  def destroySut(sut: Sut) = {
    sut.activateAutopilot(false)
  }
  
  def genInitialState: Gen[State] = {
    println("New initial state created")
    new State(new RpList(Buffer.empty[ROUTE_POINT]),
        new RpList(getRoute(0, 0, 1000).asScala),
        new RouteInfo(getRoute(0, 2, 1000)))
  }
  
  def genCommand(state: State): Gen[Command] = Gen.frequency(
    (0, genRunSegment(state)),
    (1, genRunSegment(state)))

  def genRunSegment(state: State): Gen[RunSegment] = for {
    seed <- Gen.choose(-12000, 12000)
    points <- Gen.choose(4, 7)
    speed <- Gen.choose(20, 40)
  } yield {
    state.routeInfo.setRandomSeed(seed)

    val r = state.routeInfo.generateRouteWithin(
      points,
      (state.route.size match {
        case 0 => state.startRoute.toRoutePoints()
        case _ => state.route.toRoutePoints()
      }).asJava, speed / 10, 25)
      
    println("Segment size: " + r.size)
    
    RunSegment(new RpList(r.subList(state.route.size, r.size).asScala))
  }

  case class RunSegment(route: RpList) extends Command {
    type Result = Boolean
    
    def run(sut: Sut) = {
      TestData.commands += this
      sut.runSegments(route.toRoutePoints().toList)
    }

    def nextState(state: State) = {
      new State(state.route ++ route, state.startRoute, state.routeInfo)
    }

    // This command has no preconditions
    def preCondition(state: State): Boolean = true

    // This command should always succeed (never throw an exception)
    def postCondition(state: State, result: Try[Result]) =
      result == Success(true)
  }
}

/*
 * This class is used to gain a bit more control of the generated tests.
 */
class TestSuite(numTests: PosInt) extends Checkers {
  implicit override val generatorDrivenConfig =
    PropertyCheckConfiguration(
      minSuccessful = numTests,
      sizeRange = 15)

  def myCheck(prop: Prop) = {
    check(prop)
  }
}

object CarTester {  
  def main(args: Array[String]): Unit = {    
    connect("localhost", 65191)
    
//    randomDrivingTest()
//    randomGenTest()
    
    testScala(2)
//    runLastTest()
    runLastTestHdd()
    
    disconnect()
  }
  
  def connect(host: String, port: Int) {
    rcsc_connectTcp(pointerToCString(host), port)
  }
  
  def disconnect() {
    rcsc_disconnectTcp()
  }
  
  def testScala(tests: PosInt) {
    val suite = new TestSuite(tests)
    suite.myCheck(CarSpec.propertyNoShrink())
    
    // Save last test to disk to make re-running it possible
    val oos = new ObjectOutputStream(new FileOutputStream("last_test.bin"))
    oos.writeObject(TestData.commands.toList)
    oos.close
  }
  
  def runLastTestHdd() {
    val ois = new ObjectInputStream(new FileInputStream("last_test.bin"))
    val cmds = ois.readObject.asInstanceOf[List[CarSpec.Command]].toBuffer
    ois.close
    rerunTest(cmds)
  }
  
  def runLastTest() {
    val cmds = TestData.commands.clone()
    rerunTest(cmds)
  }
  
  def rerunTest(cmds: Buffer[CarSpec.Command]) {
    println("Re-running test...")
    
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
    val edgeRoute = getRoute(0, 2, 5000);
    val startRoute = getRoute(0, 0, 5000);

    val r = new RouteInfo(edgeRoute);

    for (i <- 0 to 5) {
      var rGen = startRoute
      
      var indLast = 0
      rcsc_clearRoute(0, 3, 5000)
      followRecoveryRoute(0, 1)
      rcsc_setAutopilotActive(0, true, 2000)
      for (i <- 0 to 8) {
        rGen = r.generateRouteWithin(5, rGen, r.randInRange(1.0, 5.0), 25)

        val subRoute = rGen.subList(indLast, rGen.size())
        if (subRoute.size() > 0) {
          addRoute(0, subRoute, false, false, 3, 2000)
        }
        waitUntilRouteAlmostEnded(0)
        indLast = rGen.size()
      }
      rcsc_setAutopilotActive(0, false, 2000)
    }
  }
  
  def randomGenTest() {
    val edgeRoute = getRoute(0, 2, 5000);
    val startRoute = getRoute(0, 0, 5000);
    
    var maxAttempts = 0
    var genPoints = 0
    var usedPoints = 0

    val r = new RouteInfo(edgeRoute);

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
          addRoute(0, subRoute, true, true, 3, 2000)
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
    val edgeRoute = getRoute(0, 2, 5000);
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
    val st = getCarState(0, 5000)
    println("px: " + st.px() + ", py: " + st.py())
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

    addRoute(0, a.asJava, true, true, 3, 5000)
  }
}