/* drastically simplified by Moshe Braner */

#include "SoftRF.h"
#include "Traffic.h"
#include "RF.h"
#include "global.h"
#include "Log.h"
#include "OLED.h"
#include "global.h"
#include "PVALID.h"

#include <math.h>

bool isPacketValid(ufo_t* fop) {

    static float coslat = 0;
    if (coslat == 0)  coslat = cosf(ThisAircraft.latitude * 0.0174533);

    if (fop->prevtime == 0)                             /* no previous packet to compare to */
      return true;

    float dx, dy, speed, distsq, interval, calcdist;
    speed = fop->speed * 1.852 / 3.6;                   /* knots -> m/s */
    /* approximate distance between lat/lon pairs */
    dy = 111300.0 * (fop->latitude - fop->prevlat);     /* meters */
    dx = 111300.0 * (fop->longitude - fop->prevlon) * coslat;
    distsq = (dx*dx + dy*dy);                           /* avoid the sqrt() */
    interval = fop->timestamp - fop->prevtime;
    if (speed < 2)  speed = 2;
    calcdist = speed * interval;
    if (distsq > 2*calcdist*calcdist)                   /* moved too far, bad data */
        return false;

    if (fabs(fop->altitude - fop->prevalt) > interval * 16)    /* bad altitude data */
        return false;

    return true;
}

