package rcontrolstationcomm

import RControlStationCommLibrary._
import CAR_STATE._
import org.bridj.Pointer._
import Utils._
import scala.collection.JavaConverters._

object CarTester {
  def main(args: Array[String]): Unit = {
    rcsc_connectTcp(pointerToCString("localhost"), 65191)

    val st = getCarState(0, 5000)
    println("px: " + st.px() + ", py: " + st.py())
    println("ap_route_left: " + st.ap_route_left())

    val edgeRoute = getRoute(0, 4, 5000);
    val startRoute = getRoute(0, 1, 5000);

    for (r <- edgeRoute.asScala) {
      println("[" + r.px() + ", " + r.py() + "]")
    }

    val r = new RouteInfo(edgeRoute);
    println("XMin: " + r.xMin())
    println("XMax: " + r.xMax())
    println("YMin: " + r.yMin())
    println("YMax: " + r.yMax())
    println("Len: " + r.length())

    for (i <- 0 to 5) {
      var rGen = startRoute
      
      var indLast = 0
      rcsc_clearRoute(-1, 3, 5000)
      followRecoveryRoute(0, 2)
      rcsc_setAutopilotActive(0, true, 2000)
      for (i <- 0 to 8) {
        rGen = r.generateRouteWithin(5, rGen, 2.0, 20)

        val subRoute = rGen.subList(indLast, rGen.size())
        if (subRoute.size() > 0) {
          addRoute(0, subRoute, false, false, 3, 2000)
        }
        //      Thread.sleep(200)
        waitUntilRouteAlmostEnded(0)
        indLast = rGen.size()
      }
      rcsc_setAutopilotActive(0, false, 2000)
    }

    rcsc_disconnectTcp()
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