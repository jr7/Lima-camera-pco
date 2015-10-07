#setFormatFlags(0x31) -> dateTime function fileLine
#setFormatFlags(0x30) ->          function fileLine

from Lima import Core,Pco
Core.DebParams.setFormatFlags(0x30)
Core.DebParams.setTypeFlags(0xff)
Core.DebParams.setModuleFlags(0xffff)

cam = Pco.Camera("")

i=Pco.Interface(cam)
c=Core.CtControl(i)
a = c.acquisition()
s=c.saving()

i.getNbAcquiredFrames()
i.getNbHwAcquiredFrames()
i.getStatus()

a.getTriggerMode()

a.getTriggerModeList()

a.setAcqExpoTime(.1)
a.setAcqNbFrames(100)
c.prepareAcq()
c.startAcq()

i.getNbAcquiredFrames()
i.getNbHwAcquiredFrames()

c.stopAcq()
a.getAcqExpoTime(.1)

a.getAcqMode()
a.getLatencyTime()

c.prepareAcq()
c.startAcq()

a.getAcqExpoTime()

i.getNbHwAcquiredFrames()
i.getNbAcquiredFrames()

c.stopAcq()
