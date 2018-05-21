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
    
    val route = getRoute(0, 4, 5000);
    
    for(r <- route.asScala) {
      println("[" + r.px() + ", " + r.py() + "]")
    }
    
    val a = List.fill(3)(new ROUTE_POINT)
    a(0).px(5)
    a(0).py(8)
    a(0).speed(3.1)
    a(0).time(2900)
    
    a(1).px(2)
    a(1).py(4)
    a(1).speed(3.1)
    a(1).time(2920)
    
    a(2).px(1)
    a(2).py(-8)
    a(2).speed(2.1)
    a(2).time(2980)
    
    addRoute(0, a.asJava, true, true, 3, 5000)
    
    rcsc_disconnectTcp()
  }
}