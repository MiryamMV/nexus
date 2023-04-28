// ----------------------------------------------------------------------------
// nexus | EPrejfac.cc
//
// Basic geometry of a Xe gas volume and a Cu + Steel plate to compute aprox.
//rejection factors.
//
// Miryam Martínez Vara
// ----------------------------------------------------------------------------

#include "EPrejfac.h"
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

  REGISTER_CLASS(EPrejfac, GeometryBase)

  EPrejfac::EPrejfac():
    GeometryBase(),
    world_z_ (5. * m),
    world_xy_ (5. *m),
    diam_ (1340. * mm),
    gas_length_ (1458. *mm),
    plate_length_ (12. *cm),
    pressure_(13.5 * bar),
    temperature_(293. * kelvin),
    sc_yield_(16670. * 1/MeV),
    e_lifetime_(1000. * ms),
    // copper plate holes
    hole_diam_front_  (80. * mm),
    hole_diam_rear_   (62. * mm),
    hole_length_front_(49.5 * mm),
    hole_length_rear_ (plate_length_ - hole_length_front_),
    //msg
    steel_length_(),
    specific_vertex_()

  {
    geom_navigator_ =
      G4TransportationManager::GetTransportationManager()->GetNavigatorForTracking();

    msg_ = new G4GenericMessenger(this, "/Geometry/EPrejfac/",
				  "Control commands of EPrejfac.");

    G4GenericMessenger::Command&  steel_length_cmd =
      msg_->DeclareProperty("steel_length", steel_length_,
                            "Steel Length");
    steel_length_cmd.SetParameterName("steel_length", true);
    steel_length_cmd.SetUnitCategory("Length");

    msg_->DeclarePropertyWithUnit("specific_vertex", "cm",  specific_vertex_,
      "Set generation vertex.");

  }

  EPrejfac::~EPrejfac()
  {
    delete msg_;
    delete copper_gen_;
    delete steel_gen_;
  }

  void EPrejfac::Construct()
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

    new G4PVPlacement(nullptr, G4ThreeVector(0,0,-gas_length_/2.), gas_logic_vol,
                      gas_name, world_logic_vol,false, 0, false);

  // Cu PLATE //////////////////////////////////////////////

  G4String cu_name = "Cu_PLATE";

  G4Material* cu_mat = G4NistManager::Instance()->FindOrBuildMaterial("G4_Cu");

  //2 separate plates
  G4Tubs* cu_solid_vol =
    new G4Tubs(cu_name, 0, diam_/2., (plate_length_-steel_length_)/2., 0, twopi);

  //For steel inserted in the copper.
  //G4Tubs* cu_solid_vol =
  //  new G4Tubs(cu_name, 0, diam_/2., (plate_length_-steel_length_)/2., 0, twopi);

  G4LogicalVolume* cu_logic_vol =
    new G4LogicalVolume(cu_solid_vol, cu_mat, cu_name);

    G4VisAttributes cu_col = nexus::CopperBrown();
    cu_logic_vol->SetVisAttributes(cu_col);

  //2 separate plates
    G4VPhysicalVolume* cu_phys = new G4PVPlacement(nullptr, G4ThreeVector(0,0,(plate_length_-steel_length_)/2.),
                                                   cu_logic_vol, cu_name, world_logic_vol,false, 0, false);

  // VERTEX GENERATORS
    copper_gen_ = new CylinderPointSampler2020(cu_phys);

  // Steel PLATE //////////////////////////////////////////////

  if (steel_length_ > 0) {
    G4String steel_name = "STEEL_PLATE";

    G4Material* steel_mat = materials::Steel316Ti();

    steel_mat->SetMaterialPropertiesTable(new G4MaterialPropertiesTable());

    G4Tubs* steel_solid_vol = new G4Tubs(steel_name, 0, diam_/2., steel_length_/2., 0, twopi);

    G4LogicalVolume* steel_logic_vol = new G4LogicalVolume(steel_solid_vol, steel_mat, steel_name);

    G4VisAttributes steel_col = nexus::White();
    steel_logic_vol->SetVisAttributes(steel_col);

    //2 separate plates.
    G4VPhysicalVolume* steel_phys = new G4PVPlacement(nullptr, G4ThreeVector(0,0,plate_length_-steel_length_/2.),
                                                      steel_logic_vol, steel_name, world_logic_vol,false, 0, false);

    //For steel inserted in the copper.
    //G4VPhysicalVolume* steel_phys = new G4PVPlacement(nullptr, G4ThreeVector(0,0,(plate_length_-steel_length_)/2.),
    //                                                  steel_logic_vol, steel_name, cu_logic_vol,false, 0, false);

    // VERTEX GENERATORS
      steel_gen_ = new CylinderPointSampler2020(steel_phys);
  }

  // PMT HOLE //////////////////////////////////////////////
  //GeneratePositions();
  //G4double offset = 1. * mm;

  // Using offset
  //G4Tubs* hole_front_solid = new G4Tubs("HOLE_FRONT", 0., hole_diam_front_/2.,
  //                                      (hole_length_front_ + offset)/2., 0., twopi);

  //G4Tubs* hole_rear_solid = new G4Tubs("HOLE_REAR", 0., hole_diam_rear_/2.,
  //                                     (hole_length_rear_ + 2*offset)/2., 0., twopi);

  //G4UnionSolid* hole_solid = new G4UnionSolid("HOLE", hole_front_solid, hole_rear_solid,
  //                                            0, G4ThreeVector(0., 0., (hole_length_front_
  //                                              + 2*offset + hole_length_rear_)/2.));

  // Using no offset
  //G4Tubs* hole_front_solid = new G4Tubs("HOLE_FRONT", 0., hole_diam_front_/2.,
  //                                      hole_length_front_/2., 0., twopi);

  //G4Tubs* hole_rear_solid = new G4Tubs("HOLE_REAR", 0., hole_diam_rear_/2.,
  //                                     hole_length_rear_/2., 0., twopi);

  //G4UnionSolid* hole_solid = new G4UnionSolid("HOLE", hole_front_solid, hole_rear_solid,
  //                                            0, G4ThreeVector(0., 0., (hole_length_front_
  //                                              + hole_length_rear_)/2.));

  // PMT HOLE SUBTRACTION///////////////////////////////////

  //G4ThreeVector hole_pos = pmt_positions_[0];
  //G4double transl_z = -plate_length_/2. + hole_length_front_/2. - offset/2.;
  //hole_pos.setZ(transl_z);

  //G4SubtractionSolid* full_plate_solid =
  //new G4SubtractionSolid("EP_COPPER_PLATE", cu_solid_vol, hole_solid, 0, hole_pos);

  //for (G4int i=1; i<5; i++) {
    //hole_pos = pmt_positions_[i];
    //hole_pos.setZ(transl_z);

    //full_plate_solid = new G4SubtractionSolid("EP_COPPER_PLATE", full_plate_solid, hole_solid, 0, hole_pos);}

  //G4LogicalVolume* plate_logic_vol =
      //new G4LogicalVolume(full_plate_solid, cu_mat, "EP_COPPER_PLATE");

  //With PMT holes.
      //new G4PVPlacement(nullptr, G4ThreeVector(0,0,plate_length_/2.), plate_logic_vol,
      //                  cu_name, world_logic_vol,false, 0, false);
}

void EPrejfac::GeneratePositions()
{
  /// Function that computes and stores the XY positions of PMTs in the copper plate
  G4int num_conc_circles = 4;
  G4int num_inner_pmts = 6;
  G4double x_pitch = 125 * mm;
  G4double y_pitch = 108.3 * mm;
  G4int total_positions = 0;
  G4ThreeVector position(0.,0.,0.);

  for (G4int circle=1; circle<=num_conc_circles; circle++) {
    G4double rad     = circle * x_pitch;
    G4double step    = 360.0/num_inner_pmts;

    for (G4int place=0; place<num_inner_pmts; place++) {
      G4double angle = place * step;
      position.setX(rad * cos(angle*deg));
      position.setY(rad * sin(angle*deg));
      pmt_positions_.push_back(position);
      total_positions++;
    }

    for (G4int i=1; i<circle; i++) {
      G4double start_x = (circle-(i*0.5))*x_pitch;
      G4double start_y = i*y_pitch;
      rad  = std::sqrt(std::pow(start_x, 2) + std::pow(start_y, 2));
      G4double start_angle = std::atan2(start_y, start_x)/deg;

      for (G4int place=0; place<num_inner_pmts; place++) {
        G4double angle = start_angle + place * step;
        position.setX(rad * cos(angle*deg));
        position.setY(rad * sin(angle*deg));
        pmt_positions_.push_back(position);
        total_positions++;
       }
    }
  }
}

    G4ThreeVector EPrejfac::GenerateVertex(const G4String& region) const
  {
    G4ThreeVector vertex(0.,0.,0.);

    // WORLD
    if (region == "CENTER") {
      return vertex;
    }
    else if (region == "Cu_PLATE") {
      G4VPhysicalVolume *VertexVolume;
      do {
        vertex = copper_gen_->GenerateVertex("VOLUME");
        G4ThreeVector glob_vtx(vertex);
        //glob_vtx = glob_vtx + G4ThreeVector(0, 0, -GetELzCoord());
        VertexVolume = geom_navigator_->LocateGlobalPointAndSetup(glob_vtx, 0, false);
      } while (VertexVolume->GetName() != region);
    }
    else if (region == "STEEL_PLATE") {
      G4VPhysicalVolume *VertexVolume;
      do {
        vertex = steel_gen_->GenerateVertex("VOLUME");
        G4ThreeVector glob_vtx(vertex);
        //glob_vtx = glob_vtx + G4ThreeVector(0, 0, -GetELzCoord());
        VertexVolume = geom_navigator_->LocateGlobalPointAndSetup(glob_vtx, 0, false);
      } while (VertexVolume->GetName() != region);
    }
    else if (region == "AD_HOC") {
      return specific_vertex_;
    }
    else {
      G4Exception("[EPrejfac]", "GenerateVertex()", FatalException,
		  "Unknown vertex generation region!");
    }
    return vertex;
  }

} // end namespace nexus
