###############################################################################
# @file       UAVObjects.inc
# @author     The OpenPilot Team, http://www.openpilot.org, Copyright (C) 2011.
# @author     Tau Labs, http://taulabs.org, Copyright (C) 2012-2013
# @addtogroup 
# @{
# @addtogroup 
# @{
# @brief Makefile includes for UAVObject source files
###############################################################################
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
#

# These are the UAVObjects supposed to be build as part of the OpenPilot target
# (all architectures)

UAVOBJSRCFILENAMES =
UAVOBJSRCFILENAMES += accels
UAVOBJSRCFILENAMES += firmwareiapobj
UAVOBJSRCFILENAMES += flighttelemetrystats
UAVOBJSRCFILENAMES += gcstelemetrystats
UAVOBJSRCFILENAMES += nedaccel
UAVOBJSRCFILENAMES += objectpersistence
UAVOBJSRCFILENAMES += overosyncstats
UAVOBJSRCFILENAMES += systemalarms
UAVOBJSRCFILENAMES += systemsettings
UAVOBJSRCFILENAMES += systemstats
UAVOBJSRCFILENAMES += taskinfo
UAVOBJSRCFILENAMES += magbias
UAVOBJSRCFILENAMES += sensorsettings
UAVOBJSRCFILENAMES += inssettings
UAVOBJSRCFILENAMES += attitudesettings
UAVOBJSRCFILENAMES += gyros
UAVOBJSRCFILENAMES += gyrosbias
UAVOBJSRCFILENAMES += magnetometer
UAVOBJSRCFILENAMES += baroaltitude
UAVOBJSRCFILENAMES += baroairspeed
UAVOBJSRCFILENAMES += airspeedsettings
UAVOBJSRCFILENAMES += airspeedactual
UAVOBJSRCFILENAMES += watchdogstatus
UAVOBJSRCFILENAMES += flightstatus
UAVOBJSRCFILENAMES += modulesettings
UAVOBJSRCFILENAMES += hwdiscoveryf4
UAVOBJSRCFILENAMES += receiveractivity
UAVOBJSRCFILENAMES += gcsreceiver
UAVOBJSRCFILENAMES += faultsettings
UAVOBJSRCFILENAMES += sessionmanaging
UAVOBJSRCFILENAMES += gpsposition
UAVOBJSRCFILENAMES += gpssatellites
UAVOBJSRCFILENAMES += gpstime
UAVOBJSRCFILENAMES += gpsvelocity
UAVOBJSRCFILENAMES += actuatorcommand
UAVOBJSRCFILENAMES += actuatordesired
UAVOBJSRCFILENAMES += actuatorsettings
UAVOBJSRCFILENAMES += attitudeactual
UAVOBJSRCFILENAMES += insstate
UAVOBJSRCFILENAMES += nedposition
UAVOBJSRCFILENAMES += positionactual
UAVOBJSRCFILENAMES += stateestimation
UAVOBJSRCFILENAMES += velocityactual
UAVOBJSRCFILENAMES += homelocation
UAVOBJSRCFILENAMES += mixersettings
UAVOBJSRCFILENAMES += manualcontrolcommand
UAVOBJSRCFILENAMES += manualcontrolsettings
UAVOBJSRCFILENAMES += accessorydesired
UAVOBJSRCFILENAMES += cameradesired
UAVOBJSRCFILENAMES += stabilizationdesired
UAVOBJSRCFILENAMES += stabilizationsettings
UAVOBJSRCFILENAMES += pathdesired
UAVOBJSRCFILENAMES += tabletinfo
UAVOBJSRCFILENAMES += loitercommand
UAVOBJSRCFILENAMES += altitudeholddesired
UAVOBJSRCFILENAMES += altitudeholdsettings
UAVOBJSRCFILENAMES += relaytuning
UAVOBJSRCFILENAMES += relaytuningsettings

UAVOBJSRCFILENAMES += i2cvm
UAVOBJSRCFILENAMES += i2cvmuserprogram
UAVOBJSRCFILENAMES += vibrationanalysissettings
UAVOBJSRCFILENAMES += vibrationanalysisoutput
UAVOBJSRCFILENAMES += trimangles
UAVOBJSRCFILENAMES += trimanglessettings

UAVOBJSRC = $(foreach UAVOBJSRCFILE,$(UAVOBJSRCFILENAMES),$(OPUAVSYNTHDIR)/$(UAVOBJSRCFILE).c )
UAVOBJDEFINE = $(foreach UAVOBJSRCFILE,$(UAVOBJSRCFILENAMES),-DUAVOBJ_INIT_$(UAVOBJSRCFILE) )
