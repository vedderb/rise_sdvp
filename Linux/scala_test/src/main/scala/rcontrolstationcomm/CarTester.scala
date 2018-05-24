package rcontrolstationcomm

import RControlStationCommLibrary._
import CAR_STATE._
import org.bridj.Pointer._
import Utils._
import scala.collection.JavaConverters._

import org.scalacheck.Gen
import org.scalacheck.commands.Commands
import util.{Try,Success,Failure}

case class CarState (
    id: Int,
    connected: Boolean
)

class Car {
  def connect () {
    rcsc_connectTcp(pointerToCString("localhost"), 65191)
  }
  
  def disconnect() {
    rcsc_disconnectTcp()
  }
  
  def followRecoveryRoute() : Boolean = {
    // TODO
    true
  }
}

object CarSpec extends Commands {
  type State = CarState
  type Sut = Car

  def canCreateNewSut(newState: State, initSuts: Traversable[State],
                      runningSuts: Traversable[Sut]) = {
    initSuts.isEmpty && runningSuts.isEmpty
  }
  
  def initialPreCondition(state: State): Boolean = {
    state.connected == true
  }
  
  def newSut(state: State): Sut = {
    val sut = new Sut
    sut.connect()
    sut
  }
  
  def destroySut(sut: Sut): Unit = sut.disconnect()
  
  def genInitialState: Gen[State] = {
    new State(0, true)
  }
  
  def genCommand(state: State): Gen[Command] = Gen.oneOf(
    FollowRecovery, FollowRecovery
  )
  
  case object FollowRecovery extends Command {
    type Result = Boolean
    
    def run(sut: Sut): Result = sut.followRecoveryRoute()

    def nextState(state: State): State = state

    // This command has no preconditions
    def preCondition(state: State): Boolean = true

    // This command should always succeed (never throw an exception)
    def postCondition(state: State, result: Try[Result]) =
      result == Success(true)
  }
}

object CarTester {
  def main(args: Array[String]): Unit = {
    rcsc_connectTcp(pointerToCString("localhost"), 65191)

    randomDrivingTest()
//    randomGenTest()

    rcsc_disconnectTcp()
    
//    CarSpec.property().check()
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