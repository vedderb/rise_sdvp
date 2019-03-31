name := "RcCar Test"

version := "0.1"

scalaVersion := "2.12.6"

libraryDependencies ++= Seq(
  "org.scalacheck" % "scalacheck_2.12" % "1.14.0",
  "org.scalatest" % "scalatest_2.12" % "3.0.5",
  "com.nativelibs4java" % "bridj" % "0.6.2"
)

fork := true

initialCommands in console := """
	import rcontrolstationcomm._
	CarTester.connect("localhost", 65191)
	"""

