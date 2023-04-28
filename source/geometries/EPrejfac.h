// ----------------------------------------------------------------------------
// nexus | EPrejfac.h
//
// Basic geometry of a Xe gas volume and a Cu + Steel plate to compute aprox.
//rejection factors.
//
// Miryam Martínez Vara
// ----------------------------------------------------------------------------

#ifndef EP_REJ_FAC_H
#define EP_REJ_FAC_H

#include "GeometryBase.h"

#include <vector>
#include <G4Navigator.hh>

class G4Material;
class G4OpticalSurface;
class G4GenericMessenger;

namespace nexus {

  class CylinderPointSampler2020;

  class EPrejfac: public GeometryBase
  {
  public:
    /// Constructor
    EPrejfac();
    /// Destructor
    ~EPrejfac();

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
    G4double plate_length_;
    G4double pressure_;
    G4double temperature_;
    G4double sc_yield_;
    G4double e_lifetime_;
    G4double hole_diam_front_;
    G4double hole_diam_rear_;
    G4double hole_length_front_;
    G4double hole_length_rear_;

    //Messenger for configuration parameters
    G4GenericMessenger* msg_;
    G4double steel_length_;
    G4ThreeVector specific_vertex_;

    std::vector<G4ThreeVector> pmt_positions_;

    // Geometry Navigator
    G4Navigator* geom_navigator_;

    // Vertex generators
    CylinderPointSampler2020* copper_gen_;
    CylinderPointSampler2020* steel_gen_;

  };

} // end namespace nexus

#endif
