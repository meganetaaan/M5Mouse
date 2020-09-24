#!/bin/sh

PROJ=m5mouse

mv $PROJ{-F_Cu.gtl,.GTL}
mv $PROJ{-B_Cu.gbl,.GBL}
mv $PROJ{-F_SilkS.gto,.GTO}
mv $PROJ{-B_SilkS.gbo,.GBO}
mv $PROJ{-F_Mask.gts,.GTS}
mv $PROJ{-B_Mask.gbs,.GBS}
mv $PROJ{-Edge_Cuts.gm1,.GML}
mv $PROJ{.drl,.TXT}
