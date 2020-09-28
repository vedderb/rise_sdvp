XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
X   Artwork and documentation done by: Texas Instruments Norway AS  X
X   Company: Texas Instruments Norway AS                            X
X   Address: Gaustadalléen 21    0349 OSLO                          X
X                                                                   X
X                                                                   X
X                                                                   X
XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX

PROJECT : 03236
PCB NAME : CC112xEM
REVISION: 1.1
DATE: 2011-04-06
MANUFACTURER : 
QUANTITY:   See order (in panel)

Manufacturers marking: Two letter + year + week
The two letter code shall identify the manufaturer and is decided by the manufacturer. 
No logos or other identifiers are allowed.
The marking shall be in silk screen print at secondary side of the PCB, or in solder 
resist if no silk print at secondary side.

FILE:  PACKED WITH WinZIP 

PCB DESCRIPTION:4 LAYER PCB 1.24 MM nominal
      Copper        1   35 um
      Dielectric  1-2   0.4 mm (e.g. 2x Prepreg 7628 AT05 47% Resin)
      Copper        2   35 um
      Dielectric  2-3   0.3 mm (2 x 2157 49% Resin)
      Copper        3   35 um
      Dielectric  3-4   0.4 mm (e.g. 2x Prepreg 7628 AT05 47% Resin)
      Copper        4   35 um

  DE104iML or equivalent substrate (Resin contents around 45%, which gives Er=4.45@1GHz, TanD=0.016)
  Dimensions in mil (0.001 inch)
  DOUBLE SIDE SOLDER MASK,
  DOUBLE SIDE SILKSCREEN,
  8 MIL MIN TRACE WIDTH AND 8 MIL MIN ISOLATION.


                 
FILE NAME                              DESCRIPTION                                        FILE TYPE
-----------------------------------------------------------------------------------------------------
***PCB MANUFACTURING FILES:
l1.SPL                                 LAYER 1 COMPONENT SIDE/POSITIV                     EXT. GERBER
l2.SPL                                 LAYER 2 Inner layer/POSITIV                        EXT. GERBER
l3.SPL                                 LAYER 3 Inner layer/POSITIV                        EXT. GERBER
l4.SPL                                 LAYER 4 SOLDER SIDE/POSITIV                        EXT. GERBER
STOPCOMP.SPL                           SOLDER MASK COMPONENT SIDE/NEGATIVE                EXT. GERBER
STOPSOLD.SPL                           SOLDER MASK SOLDER SIDE/NEGATIVE                   EXT. GERBER
SILKCOMP.SPL                           SILKSCREEN COMPONENT SIDE/POSITIVE                 EXT. GERBER
SILKSOLD.SPL                           SILKSCREEN SOLDER SIDE/POSITIVE                    EXT. GERBER
NCDRILL.SPL                            NC DRILL THROUGH HOLE                              EXCELLON
DRILL.SPL                              DRILL/MECHANICAL DRAWING                           EXT. GERBER
GERBER.REP                             DRILL/NC DRILL REPORT                              ASCII
EXT_GERBER.USR                         EXTENDED GERBER APERTURE TABLE                     ASCII
CNC.USR                                NC DRILL DEVICE FILE                               ASCII

*** ASSEMBLY FILES:
PASTCOMP.SPL                           SOLDER PAST COMPONENT SIDE/POSITIVE                EXT. GERBER
PASTSOLD.SPL                           SOLDER PAST SOLDER SIDE/POSITIVE                   EXT. GERBER
ASSYCOMP.SPL                           ASSEMBLY DRAWING COMPONENT SIDE/NEGATIVE           EXT. GERBER
ASSYSOLD.SPL                           ASSEMBLY DRAWING SOLDER SIDE/NEGATIVE              EXT. GERBER
CC1120_420_470_P_P_top.REP             PICK AND PLACE FILE COMPONENT SIDE                 ASCII
CC1120_420_470_P_P_bottom.REP          PICK AND PLACE FILE SOLDER SIDE                    ASCII

*** DESIGN FILES
CC112xEM_420_470_schematic.PDF         SCHEMATIC
CC112xEM_420_470_LAYOUT.PDF            LAYOUT
CC1120EM_420_470_PARTLIST.XLS          PARTLIST IN EXCEL FORMAT

README.TXT                             THIS FILE                                          ASCII

END.


Release history
--------------------------------------------------------------------------------------------------------------
1.0.0 : 4-layer design
1.0.1 : Component changes
1.1.0 : changed footprint for L192, L201. Increases drill hole (to 1.5mm) for SMD_Socket 2x10
