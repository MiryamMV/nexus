// ----------------------------------------------------------------------------
// nexus | Xe_cylinder.cc
//
// Basic geometry of a Xe gas volume for the mass model.
//
// Miryam Martínez Vara
// ----------------------------------------------------------------------------

#include "Xe_cylinder.h"
#include "FactoryBase.h"

#include "MaterialsList.h"
#include "IonizationSD.h"
#include "OpticalMaterialProperties.h"
#include "Visibilities.h"
#include "CylinderPointSampler2020.h"

#include <G4Box.hh>
#include <G4Tubs.hh>
#include <G4SubtractionSolid.hh>
#include <G4UnionSolid.hh>
#include <G4GenericMessenger.hh>
#include <G4Orb.hh>
#include <G4NistManager.hh>
#include <G4LogicalVolume.hh>
#include <G4PVPlacement.hh>
#include <G4VPhysicalVolume.hh>
#include <G4TransportationManager.hh>
#include <G4Material.hh>
#include <G4ThreeVector.hh>
#include <G4VisAttributes.hh>
#include <G4Colour.hh>
#include <G4SDManager.hh>
#include <G4UserLimits.hh>
#include <Randomize.hh>
#include <G4OpticalSurface.hh>
#include <G4LogicalSkinSurface.hh>
#include <G4RotationMatrix.hh>
#include <G4SystemOfUnits.hh>

namespace nexus {

  REGISTER_CLASS(Xe_cylinder, GeometryBase)

  Xe_cylinder::Xe_cylinder():
    GeometryBase(),
    world_z_ (5. * m),
    world_xy_ (5. *m),
    diam_ (2600. * mm),
    gas_length_ (2600. *mm),
    pressure_(15. * bar),
    temperature_(293. * kelvin),
    sc_yield_(16670. * 1/MeV),
    e_lifetime_(1000. * ms),
    //msg
    specific_vertex_(),
    max_step_size_ (1. * mm)

  {
    geom_navigator_ =
      G4TransportationManager::GetTransportationManager()->GetNavigatorForTracking();

    msg_ = new G4GenericMessenger(this, "/Geometry/Xe_cylinder/",
				  "Control commands of Xe_cylinder.");

    msg_->DeclarePropertyWithUnit("specific_vertex", "cm",  specific_vertex_,
      "Set generation vertex.");

    msg_->DeclareProperty("max_step_size", max_step_size_, "Maximum Step Size");

  }

  Xe_cylinder::~Xe_cylinder()
  {
    delete msg_;
  }

  void Xe_cylinder::Construct()
  {

  // WORLD /////////////////////////////////////////////////

  G4String world_name = "WORLD";

  G4Material* world_mat = G4NistManager::Instance()->FindOrBuildMaterial("G4_AIR");

  world_mat->SetMaterialPropertiesTable(opticalprops::Vacuum());

  G4Box* world_solid_vol =
    new G4Box(world_name, world_xy_/2., world_xy_/2., world_z_/2.);

  G4LogicalVolume* world_logic_vol =
    new G4LogicalVolume(world_solid_vol, world_mat, world_name);
  world_logic_vol->SetVisAttributes(G4VisAttributes::GetInvisible());
  GeometryBase::SetLogicalVolume(world_logic_vol);

  // XENON /////////////////////////////////////////////////

  G4String gas_name = "XENON_GAS";

  G4Material* gas_mat = materials::GXe(pressure_, temperature_);
  gas_mat->SetMaterialPropertiesTable(opticalprops::GXe(pressure_,temperature_,sc_yield_,
                                                        e_lifetime_));

  G4Tubs* gas_solid_vol =
    new G4Tubs(gas_name, 0, diam_/2., gas_length_/2., 0, twopi);

  G4LogicalVolume* gas_logic_vol =
    new G4LogicalVolume(gas_solid_vol, gas_mat, gas_name);

  G4VisAttributes gas_col = nexus::LightBlue();
  gas_logic_vol->SetVisAttributes(gas_col);

  // creo que esta linea de codigo esta mal, debería estar posicionado en 0,0,0.
  //G4VPhysicalVolume* xe_phys =  new G4PVPlacement(nullptr, G4ThreeVector(0,0,-gas_length_/2.),
  G4VPhysicalVolume* xe_phys =  new G4PVPlacement(nullptr, G4ThreeVector(0,0,0),
                                                  gas_logic_vol, gas_name, world_logic_vol,
                                                  false, 0, false);

  /// Limit the step size in this volume for better tracking precision
  gas_logic_vol->SetUserLimits(new G4UserLimits(max_step_size_));

  /// Set the volume as an ionization sensitive detector
  IonizationSD* ionisd = new IonizationSD("/XE_CYLINDER");
  gas_logic_vol->SetSensitiveDetector(ionisd);
  G4SDManager::GetSDMpointer()->AddNewDetector(ionisd);

  // VERTEX GENERATOR
    xe_gen_ = new CylinderPointSampler2020(xe_phys);

  }

    G4ThreeVector Xe_cylinder::GenerateVertex(const G4String& region) const
  {
    G4ThreeVector vertex(0.,0.,0.);

    // WORLD
    if (region == "CENTER") {
      return vertex;
    }
    else if (region == "XENON_GAS") {
      G4VPhysicalVolume *VertexVolume;
      do {
        vertex = xe_gen_->GenerateVertex("VOLUME");
        G4ThreeVector glob_vtx(vertex);
        VertexVolume = geom_navigator_->LocateGlobalPointAndSetup(glob_vtx, 0, false);
      } while (VertexVolume->GetName() != region);
    }
    else if (region == "AD_HOC") {
      return specific_vertex_;
    }
    else {
      G4Exception("[Xe_cylinder]", "GenerateVertex()", FatalException,
		  "Unknown vertex generation region!");
    }
    return vertex;
  }

} // end namespace nexus
