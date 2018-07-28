/**********************************************************************

  Audacity: A Digital Audio Editor

  ExportDSPADPCM.h

  Jack Andersen

**********************************************************************/

#ifndef __AUDACITY_EXPORTDSPADPCM__
#define __AUDACITY_EXPORTDSPADPCM__

class ExportPlugin;

/** The only part of this class which is publically accessible is the
 * factory method New_ExportDSPADPCM() which creates a new ExportDSPADPCM object and
 * returns a pointer to it. The rest of the class declaration is in ExportDSPADPCM.cpp
 */
std::unique_ptr<ExportPlugin> New_ExportDSPADPCM();

#endif

