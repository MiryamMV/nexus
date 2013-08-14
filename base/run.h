/*   
 *  NEXUS: run
 *     - Created on: Feb 2007
 *     - Last modification: June 2007, JMALBOS
 * 
 *  ---
 *  Copyright (C) 2007 - J. Martin-Albo, J. Mu�oz
 * 
 *  This file is part of NEXUS.
 *
 *  NEXUS is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  NEXUS is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with NEXUS; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *  ---
 */

#ifndef __RUN_H__
#define __RUN_H__ 1


#include "G4UserRunAction.hh"
#include "globals.hh"
#include <bhep/messenger.h>


namespace nexus {

  class run : public G4UserRunAction {
    
  public:
    
    // Ctor/Dtor
    run(bhep::prlevel vl=bhep::NORMAL);
    ~run() {}
  
    // G4 mandatory methods 
    void BeginOfRunAction(const G4Run*);
    void EndOfRunAction(const G4Run*);

  private:

    bhep::messenger __msg;
  };

} // namespace nexus

#endif