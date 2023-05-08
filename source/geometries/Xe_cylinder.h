// ----------------------------------------------------------------------------
// nexus | Xe_cylinder.h
//
// Basic geometry of a Xe gas volume for the mass model.
//
// Miryam Martínez Vara
// ----------------------------------------------------------------------------

#ifndef XE_CILINDER_H
#define XE_CILINDER_H

#include "GeometryBase.h"

#include <vector>
#include <G4Navigator.hh>

class G4Material;
class G4OpticalSurface;
class G4GenericMessenger;

namespace nexus {

  class CylinderPointSampler2020;

  class Xe_cylinder: public GeometryBase
  {
  public:
    /// Constructor
    Xe_cylinder();
    /// Destructor
    ~Xe_cylinder();

    /// Return vertex within region <region> of the chamber
    G4ThreeVector GenerateVertex(const G4String& region) const;


    void Construct();

  private:
    void GeneratePositions();

  private:
    // Dimensions
    G4double world_z_;
    G4double world_xy_;
    G4double diam_;
    G4double gas_length_;
    G4double pressure_;
    G4double temperature_;
    G4double sc_yield_;
    G4double e_lifetime_;

    //Messenger for configuration parameters
    G4GenericMessenger* msg_;
    G4ThreeVector specific_vertex_;
    //Step size
    G4double max_step_size_;

    // Geometry Navigator
    G4Navigator* geom_navigator_;

    // Vertex generators
    CylinderPointSampler2020* xe_gen_;

  };

} // end namespace nexus

#endif
