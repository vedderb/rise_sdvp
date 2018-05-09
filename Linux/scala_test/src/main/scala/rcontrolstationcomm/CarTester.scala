package rcontrolstationcomm

import RControlStationCommLibrary._
import CAR_STATE._
import org.bridj.Pointer._

object CarTester {
  def main(args: Array[String]): Unit = {
    rcsc_connectTcp(pointerToCString("localhost"), 65191)
    val st = new CAR_STATE
    rcsc_getState(0, pointerTo(st), 1000)
    println("px: " + st.px() + ", py: " + st.py())
    rcsc_disconnectTcp()
  }
}